// Return the classification of the packet if already available.
// Compute and store it otherwise.
// If an unknown flow is received and the module
// is already analyzing, the packet record will be saved
// and analysis will be started when the next packet of this flow is received.
// flow_class = 0 means unknown flow

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"
`include "../svm/config/svm_parameters.v"

module packet_classifier
    (
        // Per packet input
        input [`FLOW_ID_WIDTH-1:0] in_flow_id,
        input [`PKT_LENGTH_WIDTH-1:0] in_length,
        input in_wr,
        output in_ready,
        // Class of the last packet received
        output reg [`FLOW_CLASS_WIDTH-1:0] out_flow_class,
        output reg [`SVM_PARAM_WIDTH*`SVM_PARAM_COUNT-1:0] out_lengths, // Just for debug
        output reg [`FLOW_PKT_COUNTER_WIDTH-1:0] out_pkt_count, // Just for debug
        output reg out_wr,

        // Standard inputs
        input clk,
        input reset

        // Interface to SRAM (if any)
        `ifdef NETFPGA_1G
            ,
            // NetFPGA 1G SRAM interface
            output [`FLOW_RAM_ADDR_WIDTH-1:0]     wr_0_addr,
            output                                wr_0_req,
            input                                 wr_0_ack,
            output [`FLOW_RAM_WORD_WIDTH-1:0]     wr_0_data,
            input                                 rd_0_ack,
            input  [`FLOW_RAM_WORD_WIDTH-1:0]     rd_0_data,
            input                                 rd_0_vld,
            output [`FLOW_RAM_ADDR_WIDTH-1:0]     rd_0_addr,
            output                                rd_0_req
        `endif
        `ifdef COMBOV2_10G2
            ,
            // Combov2 RAM interfaces
            output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_0_rd_addr,
            output                                qdr_0_rd_req,
            input                                 qdr_0_rd_rdy,
            input [`FLOW_RAM_WORD_WIDTH-1:0]      qdr_0_rd_data,
            input                                 qdr_0_rd_dv,
            output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_0_wr_addr,
            output                                qdr_0_wr_req,
            input                                 qdr_0_wr_rdy,
            output [`FLOW_RAM_WORD_WIDTH-1:0]     qdr_0_wr_data,
            output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_1_rd_addr,
            output                                qdr_1_rd_req,
            input                                 qdr_1_rd_rdy,
            input [`FLOW_RAM_WORD_WIDTH-1:0]      qdr_1_rd_data,
            input                                 qdr_1_rd_dv,
            output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_1_wr_addr,
            output                                qdr_1_wr_req,
            input                                 qdr_1_wr_rdy,
            output [`FLOW_RAM_WORD_WIDTH-1:0]     qdr_1_wr_data
        `endif
    );

    //--------------------- Internal Parameters------------------------
    // 2 FSMs: ChecK and ANalyse
	localparam STATE_RESET = 3'd0;
	localparam STATE_ANSWER = 3'd1;
	localparam STATE_STORE_LENGTH = 3'd2;
	localparam STATE_WRITE = 3'd3;
	localparam STATE_WRITE_WAIT = 3'd4;
	localparam STATE_WAIT_FOR_CACHE = 3'd5;
	// Special values of the packet counter to indicate a packet has been sent to detection
	localparam FLOW_STATE_SENT = `SVM_PACKETS_SIZE_OFFSET + `SVM_PARAM_COUNT + 1;
    localparam FLOW_STATE_DONE = `SVM_PACKETS_SIZE_OFFSET + `SVM_PARAM_COUNT + 2;

    //---------------------- Wires/Regs -------------------------------

	// Memory connections (+ FIFO management)
    wire ram_out_ready;
    wire [`FLOW_RAM_EDIT_ID_WIDTH-1:0] ram_out_edit_id_tmp, ram_out_edit_id, ram_out_edit_id_real;
    wire ram_out_new, ram_out_new_none, ram_out_new_tmp;
    wire ram_out_found_tmp, ram_out_found, ram_out_found_real;
	wire ram_in_ready;
	wire ram_in_en;
    wire [`FLOW_RAM_EDIT_ID_WIDTH-1:0] ram_in_edit_id;
	wire [`SVM_CLASS_WIDTH-1:0] ram_in_class, ram_out_class_tmp, ram_out_class, ram_out_class_real;
	wire [`FLOW_PKT_COUNTER_WIDTH-1:0] ram_in_pkt_count, ram_out_pkt_count_tmp, ram_out_pkt_count, ram_out_pkt_count_real;
	wire [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT-1:0] ram_in_lengths, ram_out_lengths_tmp, ram_out_lengths, ram_out_lengths_real;

	// detection module connections
	wire det_wr;
	wire det_new_result;
	wire det_ready;
	reg det_ready_delayed;

	// fifo in, out and ram connections
	reg fifo_in_wr;
	wire fifo_out_wr;
	wire fifo_in_rd, fifo_out_rd;
    reg fifo_ram_rd;
	wire fifo_in_full, fifo_out_full, fifo_ram_full;
	wire fifo_in_empty, fifo_ram_empty;

	// Control vars
	reg [2:0] state, state_nxt;
    reg send_out;
	reg store_ram;
	reg store_length;
	reg write_ram;
	reg send_to_det;

	// Data storage for the FSM
	reg [`FLOW_RAM_EDIT_ID_WIDTH-1:0] edit_id;
	wire [`SVM_PARAM_WIDTH-1:0] length;
	wire [`FLOW_ID_WIDTH-1:0] ram_flow_id;
	reg [`FLOW_ID_WIDTH-1:0] flow_id;
	reg [`SVM_CLASS_WIDTH-1:0] class;
	reg [`FLOW_PKT_COUNTER_WIDTH-1:0] pkt_count;
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT-1:0] lengths;

	// Data storage for the detection process
	wire write_ram_out_det;
    reg det_new_result_reg;
    reg ram_in_ready_delayed;
	wire [`FLOW_RAM_EDIT_ID_WIDTH-1:0] edit_id_in_det, edit_id_out_det;
	wire [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT-1:0] lengths_in_det, lengths_out_det;
	wire [`SVM_CLASS_WIDTH-1:0] class_out_det;
	wire [`FLOW_PKT_COUNTER_WIDTH-1:0] pkt_count_out_det;

    // Local cache of the last flow id and data
    // to avoid problems with the delay to write flow data to memory
    // 2 levels of cache: adjusted experimentally on Combov2
    reg write_to_cache;
    reg [`FLOW_ID_WIDTH-1:0] last_flow_id [0:1];
	reg [`SVM_CLASS_WIDTH-1:0] last_class [0:1];
	reg [`FLOW_PKT_COUNTER_WIDTH-1:0] last_pkt_count [0:1];
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT-1:0] last_lengths [0:1];
    reg [`FLOW_RAM_EDIT_ID_WIDTH-1:0] last_edit_id [0:1];
    reg read_from_cache [0:1];

    //----------------------- Modules ---------------------------------
    // Memory that stores data about flows
    // Pipelined, used as direct input
    // IMPORTANT assumption: read_data is 0 if no data was found
	flow_storage ram
    (
		.clk(clk),
		.reset(reset),
        .read_ready(ram_out_ready),
        .read_en(in_wr),
        .read_id(in_flow_id),
        .read_edit_id(ram_out_edit_id_tmp),
        .read_data({ram_out_class_tmp, ram_out_pkt_count_tmp, ram_out_lengths_tmp}),
        .read_data_new(ram_out_new_tmp),
        .read_data_found(ram_out_found_tmp),
		.write_ready(ram_in_ready),
		.write_en(ram_in_en),
        .write_edit_id(ram_in_edit_id),
        .write_data({ram_in_class, ram_in_pkt_count, ram_in_lengths})
        // SRAM connections (if any)
        `ifdef NETFPGA_1G
            ,
            // NetFPGA 1G SRAM connections
            .wr_0_addr          (wr_0_addr),
            .wr_0_req           (wr_0_req),
            .wr_0_ack           (wr_0_ack),
            .wr_0_data          (wr_0_data),
            .rd_0_ack           (rd_0_ack),
            .rd_0_data          (rd_0_data),
            .rd_0_vld           (rd_0_vld),
            .rd_0_addr          (rd_0_addr),
            .rd_0_req           (rd_0_req)
        `endif
        `ifdef COMBOV2_10G2
            ,
            // Combov2 RAM connections
            .qdr_0_rd_addr      (qdr_0_rd_addr),
            .qdr_0_rd_req       (qdr_0_rd_req),
            .qdr_0_rd_rdy       (qdr_0_rd_rdy),
            .qdr_0_rd_data      (qdr_0_rd_data),
            .qdr_0_rd_dv        (qdr_0_rd_dv),
            .qdr_0_wr_addr      (qdr_0_wr_addr),
            .qdr_0_wr_req       (qdr_0_wr_req),
            .qdr_0_wr_rdy       (qdr_0_wr_rdy),
            .qdr_0_wr_data      (qdr_0_wr_data),
            .qdr_1_rd_addr      (qdr_1_rd_addr),
            .qdr_1_rd_req       (qdr_1_rd_req),
            .qdr_1_rd_rdy       (qdr_1_rd_rdy),
            .qdr_1_rd_data      (qdr_1_rd_data),
            .qdr_1_rd_dv        (qdr_1_rd_dv),
            .qdr_1_wr_addr      (qdr_1_wr_addr),
            .qdr_1_wr_req       (qdr_1_wr_req),
            .qdr_1_wr_rdy       (qdr_1_wr_rdy),
            .qdr_1_wr_data      (qdr_1_wr_data)
        `endif
	);

    // FIFO to keep input data before flow_storage returns a value
    fallthrough_small_fifo
	#(
		.WIDTH(`SVM_PARAM_WIDTH + `FLOW_ID_WIDTH),
		.MAX_DEPTH_BITS(`FLOW_RAM_LATENCY_BITS)
	)
	fifo_ram
   (	// Length is reduced if SVM width is smaller than flow length width
        .din           ({in_length[`SVM_PARAM_WIDTH-1:0], in_flow_id}),
		.wr_en         (in_wr),
		.rd_en         (fifo_ram_rd),
		.dout          ({length, ram_flow_id}),
		.nearly_full   (fifo_ram_full),
		.reset         (reset),
		.clk           (clk)
	);

    // FIFO to keep flow_storage output while the FSM is not ready
    fallthrough_small_fifo
	#(
		.WIDTH(`SVM_CLASS_WIDTH + `FLOW_PKT_COUNTER_WIDTH + `SVM_PARAM_WIDTH * `SVM_PARAM_COUNT + 1 + `FLOW_RAM_EDIT_ID_WIDTH),
		.MAX_DEPTH_BITS(`FLOW_RAM_LATENCY_BITS)
	)
	fifo_ram_out
   (	// Length is reduced if SVM width is smaller than flow length width
        .din           ({ram_out_class_tmp, ram_out_pkt_count_tmp, ram_out_lengths_tmp, ram_out_found_tmp, ram_out_edit_id_tmp}),
		.wr_en         (ram_out_new_tmp),
		.rd_en         (fifo_ram_rd),
		.dout          ({ram_out_class_real, ram_out_pkt_count_real, ram_out_lengths_real, ram_out_found_real, ram_out_edit_id_real}),
		.empty         (ram_out_new_none),
		.reset         (reset),
		.clk           (clk)
	);

	// Fifo to wait for SVM detection to be ready
	fallthrough_fast_fifo
	#(
		.WIDTH(`FLOW_RAM_EDIT_ID_WIDTH + `SVM_PARAM_WIDTH * `SVM_PARAM_COUNT),
		.MAX_DEPTH_BITS(2)
	)
	fifo_in_det
   (	.din           ({edit_id, lengths}),
		.wr_en         (fifo_in_wr),
		.rd_en         (det_wr),
		.dout          ({edit_id_in_det, lengths_in_det}),
		.full          (fifo_in_full),
		.empty         (fifo_in_empty),
		.reset         (reset),
		.clk           (clk)
	);

	// Fifo to keep data while SVM detection is running
	// and while the flow RAM is not ready to get the results.
	// Should keep more values than can be computed on the same time
	// in the detection block.
	fallthrough_small_fifo
	#(
		.WIDTH(`FLOW_RAM_EDIT_ID_WIDTH + `SVM_PARAM_WIDTH * `SVM_PARAM_COUNT),
		.MAX_DEPTH_BITS(3)
	)
	fifo_out_det
   (	.din           ({edit_id_in_det, lengths_in_det}),
		.wr_en         (det_wr),
		.rd_en         (write_ram_out_det),
		.dout          ({edit_id_out_det, lengths_out_det}),
		.full          (fifo_out_full),
		.reset         (reset),
		.clk           (clk)
	);

	// SVM detection: return the class from the lengths
    // class_out_det is kept until the next result is ready:
    // it should be saved before that.
    // If the ram memory is too slow to answer, the class will be overriden.
    // and the whole packet classifier will not work properly any more.
	svm_detection svm_detector
	(
		 .x					(lengths_in_det),
		 .data_valid		(det_wr),

		 .class				(class_out_det),
		 .new_result		(det_new_result),
		 .ready				(det_ready),

		 .clk				(clk),
		 .reset				(reset)
	);

   //----------------------- Logic ---------------------------------

	// FSM logic
    assign in_ready = ram_out_ready && !fifo_ram_full;

	// Memory write multiplexing (FSM and det)
	assign ram_in_en = write_ram_out_det || write_ram;
	assign ram_in_edit_id = write_ram_out_det ? edit_id_out_det : edit_id;
	assign ram_in_class = write_ram_out_det ? class_out_det : class;
	assign ram_in_pkt_count = write_ram_out_det ? pkt_count_out_det : pkt_count;
	assign ram_in_lengths = write_ram_out_det ? lengths_out_det : lengths;

    // memory read cache: decide to read from RAM or cache
    assign ram_out_class = read_from_cache[0] ? last_class[0] : (read_from_cache[1] ? last_class[1] : ram_out_class_real);
    assign ram_out_pkt_count = read_from_cache[0] ? last_pkt_count[0] : (read_from_cache[1] ? last_pkt_count[1] : ram_out_pkt_count_real);
    assign ram_out_lengths = read_from_cache[0] ? last_lengths[0] : (read_from_cache[1] ? last_lengths[1] : ram_out_lengths_real);
    assign ram_out_found = (read_from_cache[0] || read_from_cache[1]) ? 1 : ram_out_found_real;
    assign ram_out_edit_id = read_from_cache[0] ? last_edit_id[0] : (read_from_cache[1] ? last_edit_id[1] : ram_out_edit_id_real);

	// Detection logic
	assign ram_out_new = !ram_out_new_none;
	assign pkt_count_out_det = FLOW_STATE_DONE;
	// Read the fifos as fast as possible so that their output is ready
	assign det_wr = det_ready_delayed && !fifo_in_empty && !fifo_out_full;
	assign write_ram_out_det = ram_in_ready_delayed && (det_new_result || det_new_result_reg);

    /* FSM */
	/* Listen to flow_ram output */
    always @(*) begin
        state_nxt                   = state;
        send_out					= 0;
		store_length				= 0;
		store_ram				    = 0;
		write_ram				    = 0;
		send_to_det				    = 0;
        fifo_ram_rd                 = 0;
        write_to_cache              = 0;

        case(state)
            STATE_RESET: begin
				state_nxt = STATE_ANSWER;
			end

            STATE_ANSWER: begin
				// Wait for the memory to answer
				if (ram_out_new) begin
					// Output the class if any
					send_out = 1;
					// Store the values read in registers
					store_ram = 1;
					// Should we store the length or send the packet to detection?
					if (ram_out_pkt_count != FLOW_STATE_SENT && ram_out_pkt_count != FLOW_STATE_DONE) begin
						if (ram_out_pkt_count < `SVM_PACKETS_SIZE_OFFSET + `SVM_PARAM_COUNT) begin
							state_nxt = STATE_STORE_LENGTH;
						end else begin
                            if (!fifo_in_full) begin
                                send_to_det = 1;
                                state_nxt = STATE_WRITE;
                            end else begin
                                state_nxt = STATE_WAIT_FOR_CACHE;
                            end
                            // FIFOs: go to next input values
                            fifo_ram_rd = 1;
						end
					end else begin
                        // FIFOs: go to next input values
                        fifo_ram_rd = 1;
                        state_nxt = STATE_WAIT_FOR_CACHE;
                    end
				end
            end

			STATE_STORE_LENGTH: begin
				// Store the length in input in the proper register
				store_length = 1;
                state_nxt = STATE_WRITE;
                // FIFOs: go to next input values
                fifo_ram_rd = 1;
				if (pkt_count >= `SVM_PACKETS_SIZE_OFFSET + `SVM_PARAM_COUNT - 1) begin
					// If ready, send to detection
					if (!fifo_in_full)
						send_to_det = 1;
				end
			end

			STATE_WRITE: begin
                // Write in the local cache
                // immediately to be sure to save the class if it comes
                // while waiting
                write_to_cache = 1;
				// Write the local registers in RAM
				// if the detection part is not using the RAM
                // and the RAM is ready
                // Go to wait if not ready
				if (ram_in_ready_delayed && !write_ram_out_det) begin
					write_ram = 1;
					state_nxt = STATE_WAIT_FOR_CACHE;
                end else begin
                    state_nxt = STATE_WRITE_WAIT;
                end
			end

			STATE_WRITE_WAIT: begin
				// Write the local registers in RAM
				// if the detection part is not using the RAM
                // and the RAM is ready
                // Wait if not ready
				if (ram_in_ready_delayed && !write_ram_out_det) begin
					write_ram = 1;
					state_nxt = STATE_ANSWER;
                end
			end

            STATE_WAIT_FOR_CACHE: begin
                // Just wait one clock cycle before reading the
                // cache. Should go to this state if modifying
                // the cache or input immediately before going to
                // STATE_ANSWER, to make sure that read_from_cache
                // is up to date
                state_nxt = STATE_ANSWER;
            end
      endcase // case(state)
   end // always @ (*)

    /* Manage read output */
    // The class width is enlarged here if a flow class is larger than a SVM class
    // The +1 is because classes start from 0 in svm_classifier, but here 0 means unknown
    always @(posedge clk) begin
        out_wr <= send_out;
        out_flow_class <= (ram_out_found && ram_out_pkt_count == FLOW_STATE_DONE) ? (ram_out_class+1) : 0;
        out_lengths <= ram_out_lengths;
        out_pkt_count <= ram_out_pkt_count;
    end

	/* Manage CK storage in regs */
	always @(posedge clk) begin
        if (store_ram) begin
            flow_id <= ram_flow_id;
            edit_id <= ram_out_edit_id;
            class <= ram_out_class;
            pkt_count <= ram_out_pkt_count;
        end else if (store_length) begin
            if (send_to_det)
                pkt_count <= FLOW_STATE_SENT;
            else
                pkt_count <= pkt_count + 1;
        end else if (send_to_det) begin
            pkt_count <= FLOW_STATE_SENT;
        end
	end
	// Manage lengths storage
	// They must be handled separately (generate)
	generate
		genvar i;
		for (i = 0; i < `SVM_PARAM_COUNT; i = i + 1) begin: STORE_LENGTH
			always @(posedge clk) begin
				if (store_ram) begin
						lengths[(i+1)*`SVM_PARAM_WIDTH-1:i*`SVM_PARAM_WIDTH] <= ram_out_lengths[(i+1)*`SVM_PARAM_WIDTH-1:i*`SVM_PARAM_WIDTH];
				end else if (store_length) begin
					if (pkt_count == (`SVM_PACKETS_SIZE_OFFSET+i)) begin
						lengths[(i+1)*`SVM_PARAM_WIDTH-1:i*`SVM_PARAM_WIDTH] <= length;
					end
				end
			end
		end
	endgenerate

    /* Manage the local flow data cache */
    always @(posedge clk) begin
        if (reset) begin
            last_flow_id[0] <= 0;
            last_flow_id[1] <= 0;
        end else if (write_to_cache) begin
            last_flow_id[0] <= flow_id;
            last_class[0] <= class;
            last_pkt_count[0] <= pkt_count;
            last_lengths[0] <= lengths;
            last_edit_id[0] <= edit_id;
            last_flow_id[1] <= last_flow_id[0];
            last_class[1] <= last_class[0];
            last_pkt_count[1] <= last_pkt_count[0];
            last_lengths[1] <= last_lengths[0];
            last_edit_id[1] <= last_edit_id[0];
        end else if (write_ram_out_det && edit_id_out_det == last_edit_id[0]) begin
            // Special case when the classifier writes to memory the
            // data currently in cache 0
            last_class[0] <= class_out_det;
            last_pkt_count[0] <= pkt_count_out_det;
        end else if (write_ram_out_det && edit_id_out_det == last_edit_id[1]) begin
            // Special case when the classifier writes to memory the
            // data currently in cache 1
            last_class[1] <= class_out_det;
            last_pkt_count[1] <= pkt_count_out_det;
        end
        // This supposes that the comparison data is available at least
        // 1 clock cycle before the RAM data is read.
        // This is sure due to the RAM read time and FSM.
        read_from_cache[0] <= ram_flow_id == last_flow_id[0];
        read_from_cache[1] <= ram_flow_id == last_flow_id[1];
    end

	/* Delay the det_ready and send_to_det signals */
	always @(posedge clk) begin
		det_ready_delayed <= det_ready;
		fifo_in_wr <= send_to_det;
	end

	/* Manage the signals required for det to write in RAM */
    /* when the RAM is ready */
	always @(posedge clk) begin
		ram_in_ready_delayed <= ram_in_ready;
        if (reset)
            det_new_result_reg <= 0;
        else if (write_ram_out_det)
            det_new_result_reg <= 0;
        else if (det_new_result)
            det_new_result_reg <= 1;
	end

   /* Manage FSM state */
   always @(posedge clk) begin
      if(reset) begin
         state <= STATE_RESET;
      end
      else begin
         state <= state_nxt;
      end
   end

endmodule
