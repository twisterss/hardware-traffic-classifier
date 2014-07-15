/////////////////////////////////////////////////////////////////////////////
// Receives packets through the NetFPGA bus.
// Sends them
///////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"
`include "../svm/config/svm_parameters.v"

// Size of the data sent to the computer
`define RESULT_SPEC_WIDTH (32 * 2 + 16 * 2 + 8 + 8 + 16 * 3 + 8)
`define RESULT_SPEC_WORDS (`RESULT_SPEC_WIDTH / 64)
`define RESULT_SPEC_REMAINING_BYTES ((`RESULT_SPEC_WIDTH / 8) % 8)
// Number of results sent in one packet to the computer
`define RESULT_SPEC_PER_PACKET 8
// Size of 1 packet sent to the computer (should be at least 60 bytes to be sent properly!)
`define RESULT_PACKET_WIDTH (`RESULT_SPEC_WIDTH * `RESULT_SPEC_PER_PACKET)
`define RESULT_PACKET_WORDS (`RESULT_PACKET_WIDTH / 64 + (`RESULT_PACKET_WIDTH % 64 == 0 ? 0 : 1))
`define RESULT_FULL_PACKET_WORDS (`RESULT_PACKET_WORDS + 1)
`define RESULT_PACKET_LAST_BYTES (((`RESULT_PACKET_WIDTH/8)%8 == 0) ? 8 : ((`RESULT_PACKET_WIDTH/8)%8))
// Timeout before sending a partially full result packet (in cycles)
`define SEND_PACKET_TIMEOUT 12500000
// Number of variables accessible through debug registers
`define TRAFF_CLASS_DEBUG_VARS 11

module traffic_classifier
#(
    parameter DATA_WIDTH = 64,
    parameter CTRL_WIDTH = DATA_WIDTH/8,
    parameter UDP_REG_SRC_WIDTH = 2,
    parameter INPUT_ARBITER_STAGE_NUM = 2,
    parameter IO_QUEUE_STAGE_NUM = `IO_QUEUE_STAGE_NUM,
    parameter NUM_OUTPUT_QUEUES = 8,
    parameter NUM_IQ_BITS = 3,
    parameter STAGE_NUM = 4,
    parameter CPU_QUEUE_NUM = 0
)(
    // --- data path interface
    output reg [DATA_WIDTH-1:0]           out_data,
    output reg [CTRL_WIDTH-1:0]           out_ctrl,
    output reg                            out_wr,
    input                                 out_rdy,

    input  [DATA_WIDTH-1:0]               in_data,
    input  [CTRL_WIDTH-1:0]               in_ctrl,
    input                                 in_wr,
    output reg                            in_rdy,

    // --- Register interface
    input                                 reg_req_in,
    input                                 reg_ack_in,
    input                                 reg_rd_wr_L_in,
    input  [`UDP_REG_ADDR_WIDTH-1:0]      reg_addr_in,
    input  [`CPCI_NF2_DATA_WIDTH-1:0]     reg_data_in,
    input  [UDP_REG_SRC_WIDTH-1:0]        reg_src_in,

    output                                reg_req_out,
    output                                reg_ack_out,
    output                                reg_rd_wr_L_out,
    output [`UDP_REG_ADDR_WIDTH-1:0]      reg_addr_out,
    output [`CPCI_NF2_DATA_WIDTH-1:0]     reg_data_out,
    output [UDP_REG_SRC_WIDTH-1:0]        reg_src_out,
    
    // --- Interface to SRAM
    output [`FLOW_RAM_ADDR_WIDTH-1:0]     wr_0_addr,
    output                                wr_0_req,
    input                                 wr_0_ack,
    output [`FLOW_RAM_WORD_WIDTH-1:0]     wr_0_data,
    input                                 rd_0_ack,
    input  [`FLOW_RAM_WORD_WIDTH-1:0]     rd_0_data,
    input                                 rd_0_vld,
    output [`FLOW_RAM_ADDR_WIDTH-1:0]     rd_0_addr,
    output                                rd_0_req,

    // --- Misc
    input                                 clk,
    input                                 reset
);
    
    `LOG2_FUNC
   
    //--------------------- Internal Parameter-------------------------
    localparam EXTRACT_FIELDS = 3'h0;
    localparam SEND_TO_CLASSIFIER = 3'h1;
    localparam SEND_TO_CLASSIFIER_WAIT = 3'h2;
    localparam WAIT_FOR_CLASSIFIER = 3'h3;
    localparam SKIP_TO_NEXT_PACKET = 3'h4;

    //---------------------- Wires/Regs -------------------------------
    reg [2:0]                       state, state_nxt;

    reg [3:0]                       extract_cnt;
    reg                             extract_cnt_reset;
    reg                             allow_extract;
	
	reg 						    classifier_wr;
	wire							classifier_ready;
	wire [`FLOW_CLASS_WIDTH-1:0]    class;
	wire [32:0] 					lengths;	       // Used for
	wire [`FLOW_PKT_COUNTER_WIDTH-1:0] flow_pkt_count; // debug only
	wire							classifier_new_result;
    reg                             classifier_works;
    wire                            classifier_full_ready;

	// Fields read in the packet
    reg [15:0]                      ethertype;
    reg [15:0]                      ip_length;
    reg [31:0]                      src_ip, dst_ip;
    reg [7:0]                       protocol;
    reg [15:0]                      src_port, dst_port;
	wire 						    is_valid_pkt;
	
	// Values sent to the flow constructor
	wire [`FLOW_ID_WIDTH-1:0]	    flow_id;
	wire [`PKT_LENGTH_WIDTH-1:0]    pkt_length;
	
    // FSM to send data through the NetFPGA packet bus
    reg [1:0]                       send_state, send_state_nxt;
    localparam SEND_READY = 0;
    localparam SEND_ACTIVE = 1;
    localparam SEND_WAIT = 2;
    
    // Signals of the send FSM
	reg							           send_data_en;
	reg							           send_data_ready;
	wire [`RESULT_SPEC_WIDTH-1:0]          result_spec;
	reg [`RESULT_SPEC_WIDTH+7*8-1:0]       data_to_send;
    reg [log2(`RESULT_PACKET_WORDS+2)-1:0] packet_words_remaining;
    reg [log2(`RESULT_PACKET_WORDS+2)-1:0] packet_words_to_send;
    reg [2:0]                              last_bytes_to_send;
    wire [15:0]                            result_packet_words_const;
    wire [15:0]                            result_packet_bytes_const;
    reg                                    store_spec;
    reg                                    store_send_empty;
    reg [log2(`SEND_PACKET_TIMEOUT+1)-1:0] send_timeout;
    
    
    // Data available through debug registers
    reg [31:0]                               debug_write_count;
    reg [(`TRAFF_CLASS_DEBUG_VARS-1)*32-1:0] debug_data;


   //----------------------- Modules ---------------------------------
	 
	packet_classifier classifier
	(
		.in_flow_id			(flow_id),
		.in_length			(pkt_length),
		.in_wr				(classifier_wr),
		.in_ready			(classifier_ready),
		
		.out_flow_class	    (class),
		.out_lengths		(lengths),
		.out_pkt_count		(flow_pkt_count),
		.out_wr				(classifier_new_result),

        .wr_0_addr          (wr_0_addr),
        .wr_0_req           (wr_0_req),
        .wr_0_ack           (wr_0_ack),
        .wr_0_data          (wr_0_data),
        .rd_0_ack           (rd_0_ack),
        .rd_0_data          (rd_0_data),
        .rd_0_vld           (rd_0_vld),
        .rd_0_addr          (rd_0_addr),
        .rd_0_req           (rd_0_req),
	
        .clk				(clk),
		.reset				(reset)
   );
   
   
   generic_hw_regs
   #(
      .UDP_REG_SRC_WIDTH(UDP_REG_SRC_WIDTH),
      .TAG(`SVM_CLASS_BLOCK_ADDR),
      .REG_ADDR_WIDTH(`SVM_CLASS_REG_ADDR_WIDTH),
      .NUM_REGS_USED(`TRAFF_CLASS_DEBUG_VARS),
      .REG_START_ADDR(0)
   )   
   debug_regs
   (
      .reg_req_in(reg_req_in),
      .reg_ack_in(reg_ack_in),
      .reg_rd_wr_L_in(reg_rd_wr_L_in),
      .reg_addr_in(reg_addr_in),
      .reg_data_in(reg_data_in),
      .reg_src_in(reg_src_in),

      .reg_req_out(reg_req_out),
      .reg_ack_out(reg_ack_out),
      .reg_rd_wr_L_out(reg_rd_wr_L_out),
      .reg_addr_out(reg_addr_out),
      .reg_data_out(reg_data_out),
      .reg_src_out(reg_src_out),

      .hardware_regs({debug_write_count, debug_data}),

      .clk(clk),
      .reset(reset)
    );

    //----------------------- Logic ---------------------------------

    // Construct values to send
	assign flow_id = {src_ip, dst_ip, protocol, src_port, dst_port};
	assign pkt_length = ip_length[`PKT_LENGTH_WIDTH-1:0];
    // The classifier is totally ready if all its results have been sent
    assign classifier_full_ready = classifier_ready && classifier_works == 0 && send_data_ready;
	// Valid packets are IPv4 UDP or TCP packets
	assign is_valid_pkt = (ethertype == 16'h800 && (protocol == 8'h6 || protocol == 8'h11));

    /* Main FSM (manage input and classifier) */
    always @(posedge clk) begin
        if(reset) begin
            state <= SKIP_TO_NEXT_PACKET;
        end
        else begin
            state <= state_nxt;
        end
    end
    always @(*) begin
        state_nxt                  = state;
        in_rdy                     = 0;
        allow_extract              = 0;
        extract_cnt_reset          = 0;
        classifier_wr              = 0;
        send_data_en           = 0;

        case(state)
            SKIP_TO_NEXT_PACKET: begin
                // Ready to receive (except just after reset during memory reset)
                in_rdy = 1;
                // Keep reading until we reach the beginning of the next packet
                if (in_ctrl == 8'hFF && in_wr) begin
                    state_nxt = EXTRACT_FIELDS;
                    extract_cnt_reset = 1;
                end
            end
            
            EXTRACT_FIELDS: begin
                // Ready to receive
                in_rdy = 1;
                allow_extract = in_wr;
                // Jump to next state when extraction process is done
                if (extract_cnt == 4 && in_wr) begin
                    // Next state is SEND_TO_CLASSIFIER if packet is TCP or UDP
                    if (is_valid_pkt) begin
                        in_rdy = 0;
                        if (classifier_full_ready)
                            state_nxt = SEND_TO_CLASSIFIER;
                        else
                            state_nxt = SEND_TO_CLASSIFIER_WAIT;
                    end else begin
                        state_nxt = SKIP_TO_NEXT_PACKET;
                    end
                end
            end

            SEND_TO_CLASSIFIER_WAIT: begin
                // Wait for the classifier to be ready to receive
                if (classifier_full_ready)
                    state_nxt = SEND_TO_CLASSIFIER;
            end

            SEND_TO_CLASSIFIER: begin
                // enable the classifier
                classifier_wr = 1;
                // Count the number of packets sent to the classifier
                state_nxt = WAIT_FOR_CLASSIFIER;
            end

            WAIT_FOR_CLASSIFIER: begin
                // Wait for the answer of the classifier (small deterministic time)
                if (classifier_new_result) begin
                    // save the result (and packet data) in debug data
                    send_data_en = 1;
                    // go wait for next packet
                    state_nxt = SKIP_TO_NEXT_PACKET;
                end
            end
        endcase // case(state)
    end // always @ (*)

    /* Read the packet header to get known fields */
    always @(posedge clk) begin
        if (allow_extract) begin
            case (extract_cnt)
                1: begin
                    ethertype     <= in_data[31:16];
                end

                2: begin
                    ip_length     <= in_data[63:48];
                    protocol      <= in_data[7:0];
                end

                3: begin
                    src_ip        <= in_data[47:16];
                    dst_ip[31:16] <= in_data[15:0]; 
                end   

                4: begin
                    dst_ip[15:0]  <= in_data[63:48];
                    src_port      <= in_data[47:32];
                    dst_port      <= in_data[31:16];
                end
                default:   ;     
            endcase
        end
    end

    /* Manage extract_cnt */
    always @(posedge clk) begin
        if (extract_cnt_reset || reset)
            extract_cnt <= 0;
        else begin
            // Only increment extract_cnt during the extraction process
            if (allow_extract) begin
                extract_cnt <= extract_cnt + 1;
            end
        end
    end
	    
    /* Checks if the classifier is working.
       /!\ Adjust the size of the register if the classifier
       can process more than 1 job at a time */
    always @(posedge clk) begin
        if (reset)
            classifier_works <= 0;
        else if (classifier_wr && !classifier_new_result)
            classifier_works <= classifier_works + 1;
        else if (!classifier_wr && classifier_new_result)
            classifier_works <= classifier_works - 1;
    end
    
    /* FSM to send results to the NetFPGA bus */
    assign result_packet_words_const = `RESULT_PACKET_WORDS;
    assign result_packet_bytes_const = `RESULT_PACKET_WIDTH / 8;
    always @(posedge clk) begin
        if(reset) begin
            send_state <= SEND_READY;
        end
        else begin
            send_state <= send_state_nxt;
        end
    end
    always @(*) begin
        send_state_nxt             = send_state;
        send_data_ready            = 0;
        store_spec                 = 0;
        store_send_empty           = 0;
        out_wr                     = 0;
        out_data                   = 64'b0;
        out_ctrl                   = 8'b0;

        case(send_state)
            SEND_READY: begin
                if (send_data_en) begin
                    // Received a spec.
                    store_spec = 1;
                    if (out_rdy)
                        send_state_nxt = SEND_ACTIVE;
                    else
                        send_state_nxt = SEND_WAIT;
                end else if (packet_words_remaining != `RESULT_FULL_PACKET_WORDS && send_timeout >= `SEND_PACKET_TIMEOUT && !classifier_wr) begin
                    // Time out: send the packet with trailing zeroes
                    // Do not time out if something is written to the classifier
                    // to avoid that the send part becomes not ready
                    // when a result arrives.
                    store_send_empty = 1;
                    if (out_rdy)
                        send_state_nxt = SEND_ACTIVE;
                    else
                        send_state_nxt = SEND_WAIT;
                end else begin
                    send_data_ready = 1;
                end
            end
            SEND_ACTIVE: begin
                out_wr = 1;
                // Manage output
                if (packet_words_remaining == `RESULT_FULL_PACKET_WORDS) begin
                    //Sending the header
                    out_ctrl = 8'hFF;
                    out_data = {16'b10101010, result_packet_words_const, 16'd0, result_packet_bytes_const};
                end else begin
                    out_data = data_to_send[`RESULT_SPEC_WIDTH+7*8-1:`RESULT_SPEC_WIDTH+7*8-64];
                    if (packet_words_remaining == 1) begin
                        // Last word, set control accordingly
                        out_ctrl = 1<<(8-`RESULT_PACKET_LAST_BYTES);
                    end
                end
                // Choose next state
                if (packet_words_remaining == 1 || (packet_words_to_send == 1 && packet_words_remaining != 2))
                    // We keep writing if there is at least 1 more word to send
                    // or if only the last word of the packet remains to send
                    // because the last word of the packet does not need to be
                    // full to be sent
                    send_state_nxt = SEND_READY;
                else if (!out_rdy)
                    send_state_nxt = SEND_WAIT;
            end
            SEND_WAIT: begin
                if (out_rdy)
                    send_state_nxt = SEND_ACTIVE;
            end
        endcase
    end
    
    /* Counter of number of words remaining to write for a packet */
    always @(posedge clk) begin
        if (reset || (packet_words_remaining == 1 && out_wr))
            packet_words_remaining <= `RESULT_FULL_PACKET_WORDS;
        else if (out_wr)
            packet_words_remaining <= packet_words_remaining - 1;
    end
    
    /* Management of the current data to send */
    assign result_spec = {{(8-`FLOW_PKT_COUNTER_WIDTH){1'b0}}, flow_pkt_count, 5'b0, lengths[32:22], 5'b0, lengths[21:11], 5'b0, lengths[10:0], class, dst_port, src_port, protocol, dst_ip, src_ip};
    always @(posedge clk) begin
        if (reset) begin
            last_bytes_to_send <= 0;
        end else if (store_spec) begin
            // Store the generated spec to send
            if (last_bytes_to_send + `RESULT_SPEC_REMAINING_BYTES >= 8)
                packet_words_to_send <= `RESULT_SPEC_WORDS + 1;
            else
                packet_words_to_send <= `RESULT_SPEC_WORDS;
            last_bytes_to_send <= (last_bytes_to_send + `RESULT_SPEC_REMAINING_BYTES) % 8;
            // Assignment of data_to_send
            case (last_bytes_to_send)
                0:
                    data_to_send[`RESULT_SPEC_WIDTH+7*8-1:0] <= {result_spec, {(7*8){1'b0}}};
                1:
                    data_to_send[`RESULT_SPEC_WIDTH+6*8-1:0] <= {result_spec, {(6*8){1'b0}}};
                2:
                    data_to_send[`RESULT_SPEC_WIDTH+5*8-1:0] <= {result_spec, {(5*8){1'b0}}};
                3:
                    data_to_send[`RESULT_SPEC_WIDTH+4*8-1:0] <= {result_spec, {(4*8){1'b0}}};
                4:
                    data_to_send[`RESULT_SPEC_WIDTH+3*8-1:0] <= {result_spec, {(3*8){1'b0}}};
                5:
                    data_to_send[`RESULT_SPEC_WIDTH+2*8-1:0] <= {result_spec, {(2*8){1'b0}}};
                6:
                    data_to_send[`RESULT_SPEC_WIDTH+1*8-1:0] <= {result_spec, {(1*8){1'b0}}};
                7:
                    data_to_send[`RESULT_SPEC_WIDTH-1:0] <= result_spec;
            endcase
        end else if (store_send_empty) begin
            // Get ready to send zeros to fill a packet
            packet_words_to_send <= packet_words_remaining;
        end else if (out_wr && packet_words_remaining != `RESULT_FULL_PACKET_WORDS) begin
            // Writing a data word (not the header), shift the data
            packet_words_to_send <= packet_words_to_send - 1;
            data_to_send <= {data_to_send, 64'b0};
            if (packet_words_remaining == 1)
                last_bytes_to_send <= 0;
        end
    end
    
    /* Send timeout counter */
    /* A write to the classifier resets it to
    avoid that sending becomes not ready while a result is
    coming */
    always @(posedge clk) begin
        if (store_spec || classifier_wr)
            send_timeout <= 0;
        else if (send_data_ready)
            send_timeout <= send_timeout + 1;
    end
    
    /* Manage debug data (accessible through regs) */
    always @(posedge clk) begin
        if (send_data_en)
            debug_data <= {{(32-`FLOW_PKT_COUNTER_WIDTH){1'b0}}, flow_pkt_count, 21'b0, lengths[32:22], 21'b0, lengths[21:11], 21'b0, lengths[10:0], 24'b0, class, 16'b0, dst_port, 16'b0, src_port, 24'b0, protocol, dst_ip, src_ip};
        if (reset)
            debug_write_count <= 0;
        else if (out_wr)
            debug_write_count <= debug_write_count + 1;
    end
endmodule
