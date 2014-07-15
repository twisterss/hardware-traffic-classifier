// This module stores data about each flow in the SRAM available on the board.
// Flow ids are too big to be used directly as addresses.
// So an algorithm similar to CMS is used to determine addresses to update.
// Reading flow data is done in 6 clock cycles.
// Writing flow data is done in 4 clock cycles, only after a read of this flow.
// Information required to write new data for a flow (edit_id)
// is returned along with flow data during the read operation.
// read_data is guaranteed to be 0 if data has not been found (read_data_found = 0).
// The RAM used is read and written with the ALT part of the address
// changing at each operation to allow the RAM to parallelize memories internally.
// /!\ read_id input is not registered to save a clock cycle,
// connect directly to a register!

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"

module flow_storage(
    input clk,
    input reset,
    // Read interface
    output reg read_ready,
    input read_en,
    input [`FLOW_ID_WIDTH-1:0] read_id,
    output [`FLOW_RAM_EDIT_ID_WIDTH-1:0] read_edit_id,
    output reg [`FLOW_DATA_WIDTH-1:0] read_data,
    output reg read_data_new,
    output reg read_data_found,
    // Write interface
    output reg write_ready,
    input write_en,
    input [`FLOW_RAM_EDIT_ID_WIDTH-1:0] write_edit_id,
    input [`FLOW_DATA_WIDTH-1:0] write_data
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

    // Declarations

    // External RAM simplified interface connection
    wire ram_write_ready;
    wire ram_write_en;
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] ram_write_addr;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] ram_write_data;
    wire ram_read_ready;
    wire ram_read_en;
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] ram_read_addr;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] ram_read_data;
    wire ram_read_data_new;

	sram_reset_intf ram
	(
        .clk(clk),
        .reset(reset),

        .write_ready(ram_write_ready),
        .write_en(ram_write_en),
        .write_addr(ram_write_addr),
        .write_data(ram_write_data),

        .read_ready(ram_read_ready),
        .read_en(ram_read_en),
        .read_addr(ram_read_addr),
        .read_data(ram_read_data),
        .read_data_new(ram_read_data_new)

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

    // Hash functions connections
    wire [`FLOW_RAM_FULL_HASH_WIDTH-1:0] hashes_out [0:`FLOW_RAM_HASHES-1];

    generate
        genvar hash_id;
        for (hash_id = 0; hash_id < `FLOW_RAM_HASHES; hash_id = hash_id + 1) begin: hash_inst
            simple_hash #(
                .HASH_ID(hash_id)
            ) hash_function (
                .data_in(read_id),
                .data_out(hashes_out[hash_id])
            );
        end
    endgenerate

    // Timestamp generator connection
    wire [`FLOW_TIMESTAMP_WIDTH-1:0] timestamp;

    timestamp_gen #(
        .TIMESTAMP_WIDTH(`FLOW_TIMESTAMP_WIDTH),
        .TIMESTAMP_UNIT_WIDTH(`FLOW_TIMESTAMP_UNIT_WIDTH),
        .TIMESTAMP_UNIT(`FLOW_TIMESTAMP_UNIT)
    ) timer (
        .clk(clk),
        .reset(reset),
        .timestamp(timestamp)
    );

    // Bits adder connection (used for redundancy computation)
    wire [`FLOW_RAM_HASHES-1:0] sum_bits_in;
    wire [`FLOW_REDUNDANCY_WIDTH:0] sum_bits_out;

    multi_sum #(
        .VALUE_WIDTH(1),
        .VALUE_COUNT(`FLOW_RAM_HASHES),
        .SUM_WIDTH(`FLOW_REDUNDANCY_WIDTH+1)
    ) sum_bits (
        .values(sum_bits_in),
        .sum(sum_bits_out)
    );

    // FIFO connection (to store hashes while reading the memory)
    wire [`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES-1:0] read_hashes_fifo_in;
    wire [`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES-1:0] read_hashes_fifo_out;
	wire read_hashes_fifo_wr_en, read_hashes_fifo_rd_en;
    fallthrough_small_fifo
	#(
		.WIDTH(`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES),
		.MAX_DEPTH_BITS(`FLOW_RAM_LATENCY_BITS)
	)
	fifo_read_hashes
   (	.din           (read_hashes_fifo_in),
		.wr_en         (read_hashes_fifo_wr_en),
		.rd_en         (read_hashes_fifo_rd_en),
		.dout          (read_hashes_fifo_out),
		.full          (fifo_read_hashes_full),
		.reset         (reset),
		.clk           (clk)
	);

    // Variables of the read part
    reg [1:0] read_state, read_state_nxt;
    localparam READ_RESET = 0;
    localparam READ_READY = 1;
    localparam READ_SEARCH = 2;
    reg [`FLOW_RAM_FULL_HASH_WIDTH-1:0] read_hashes_saved [0:`FLOW_RAM_HASHES-1];
    wire [`FLOW_RAM_FULL_HASH_WIDTH-1:0] read_hash_saved;
    wire [`FLOW_RAM_FULL_HASH_WIDTH-1:0] read_hashes_saved_res [0:`FLOW_RAM_HASHES-1];
    wire [`FLOW_RAM_FULL_HASH_WIDTH-1:0] read_hash_saved_res;
    reg [`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES-1:0] read_hashes_merged;
    reg [`FLOW_RAM_HASHES-1:0] read_active_hashes;
    reg read_save_hash;
    reg read_search_data_to_send;
    reg read_search_ram_ready;
    reg [`FLOW_DATA_WIDTH-1:0] read_search_result;
    reg read_search_found;
    reg [`FLOW_RAM_HASHES-1:0] read_search_active_hashes;
    reg [`FLOW_REDUNDANCY_WIDTH-1:0] read_search_highest_redundancy;
    reg [`FLOW_RAM_HASH_ID_WIDTH-1:0] read_search_highest_redundancy_hash;
    reg [`FLOW_RAM_HASH_ID_WIDTH-1:0] read_search_counter, read_search_counter_res;
    reg read_search_counter_done, read_search_res_new;
    wire [`FLOW_RAM_FULL_HASH_WIDTH-`FLOW_RAM_ADDR_RAND_WIDTH-1:0] read_search_data_hash;
    wire [`FLOW_TIMESTAMP_WIDTH-1:0] read_search_data_timestamp;
    wire [`FLOW_TIMESTAMP_WIDTH-1:0] read_search_data_time_diff;
    wire [`FLOW_REDUNDANCY_WIDTH-1:0] read_search_data_redundancy;
    wire read_search_data_current;
    wire read_search_data_hash_ok;
    reg read_send_enable;

    // Variables of the write part
    reg [1:0] write_state, write_state_nxt;
    localparam WRITE_RESET = 2'd0;
    localparam WRITE_READY = 2'd1;
    localparam WRITE_WAIT = 2'd2;
    localparam WRITE_WRITE = 2'd3;
    wire [`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES-1:0] write_hashes;
    wire [`FLOW_RAM_HASHES-1:0] write_active_hashes;
    reg write_save_input;
    reg [`FLOW_RAM_FULL_HASH_WIDTH-1:0] write_hash_saved [0:`FLOW_RAM_HASHES-1];
    wire [`FLOW_RAM_FULL_HASH_WIDTH-1:0] write_hash_saved_cur;
    reg [`FLOW_RAM_HASHES-1:0] write_hash_active_saved;
    reg [`FLOW_DATA_WIDTH-1:0] write_data_saved;
    reg [`FLOW_RAM_HASH_ID_WIDTH-1:0] write_counter;
    wire [`FLOW_REDUNDANCY_WIDTH-1:0] write_redundancy;
    reg write_ram_en;
    reg write_counter_en;
    reg write_counter_reset;

    // Logic

    // Read PIPELINE

    // Input management FSM
    always @(posedge clk) begin
        if (reset)
            read_state <= READ_RESET;
        else
            read_state <= read_state_nxt;
    end
    always @(*) begin
        read_state_nxt = read_state;
        read_ready = 0;
        read_save_hash = 0;
        case (read_state)
            READ_RESET: begin
                // Wait for the RAM to be reset before acting as ready
                if (ram_read_ready)
                    read_state_nxt = READ_READY;
            end
            READ_READY: begin
                if (read_en) begin
                    read_state_nxt = READ_SEARCH;
                    read_save_hash = 1;
                end else begin
                    read_ready = !fifo_read_hashes_full;
                end
            end
            READ_SEARCH: begin
                if (read_search_counter == `FLOW_RAM_HASHES-1 && ram_read_en) begin
                    read_state_nxt = READ_READY;
                    read_ready = !fifo_read_hashes_full;
                end
            end
        endcase
    end

    // First step: hash values management
    assign read_hashes_fifo_wr_en = read_save_hash;
	assign read_hashes_fifo_rd_en = read_send_enable;
    generate
        for (hash_id = 0; hash_id < `FLOW_RAM_HASHES; hash_id = hash_id + 1) begin: hash_conn
            // Save
            always @(posedge clk) begin
                if (read_save_hash)
                    read_hashes_saved[hash_id] <= hashes_out[hash_id];
            end
            // Merge for the FIFO
            assign read_hashes_fifo_in[(hash_id+1) * `FLOW_RAM_FULL_HASH_WIDTH - 1:hash_id * `FLOW_RAM_FULL_HASH_WIDTH] = hashes_out[hash_id];
            assign read_hashes_saved_res[hash_id] = read_hashes_fifo_out[(hash_id+1) * `FLOW_RAM_FULL_HASH_WIDTH - 1:hash_id * `FLOW_RAM_FULL_HASH_WIDTH];
        end
    endgenerate

    // Second step: search: send addresses to the RAM
    assign read_hash_saved = read_hashes_saved[read_search_counter];
    // Let the ALT part of the address alternate at each clock cycle thanks to the counter
    assign ram_read_addr = {read_search_counter[`FLOW_RAM_ADDR_ALT_WIDTH-1:0], read_hash_saved[`FLOW_RAM_FULL_HASH_WIDTH - 1: `FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH]};
    assign ram_read_en = read_search_ram_ready && read_search_data_to_send;
    always @(posedge clk) begin
        if (reset) begin
            // Reset
            read_search_counter <= 0;
            read_search_data_to_send <= 0;
        end else begin
            // Is there still data to send?
            if (read_save_hash)
                read_search_data_to_send <= 1;
            else if (read_search_counter == `FLOW_RAM_HASHES-1 && ram_read_en)
                read_search_data_to_send <= 0;
            // Hash counter manager
            if (ram_read_en) begin
                if (read_search_counter == `FLOW_RAM_HASHES-1)
                    read_search_counter <= 0;
                else
                    read_search_counter <= read_search_counter + 1;
            end
            // Delay the RAM ready signal by 1 clock cycle
            read_search_ram_ready <= ram_read_ready;
        end
    end

    // Third step: search: receive values from the RAM
    assign read_search_data_hash = ram_read_data[`FLOW_RAM_WORD_WIDTH-1:`FLOW_RAM_WORD_WIDTH-(`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH)];
    assign read_search_data_timestamp = ram_read_data[`FLOW_RAM_WORD_WIDTH-(`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH)-1:`FLOW_RAM_WORD_WIDTH-(`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH + `FLOW_TIMESTAMP_WIDTH)];
    assign read_search_data_time_diff = timestamp - read_search_data_timestamp;
    assign read_search_data_redundancy = ram_read_data[`FLOW_RAM_WORD_WIDTH-(`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH + `FLOW_TIMESTAMP_WIDTH)-1:`FLOW_RAM_WORD_WIDTH-(`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH + `FLOW_TIMESTAMP_WIDTH + `FLOW_REDUNDANCY_WIDTH)];
    assign read_hash_saved_res = read_hashes_saved_res[read_search_counter_res];
    assign read_search_data_hash_ok = read_search_data_hash == read_hash_saved_res[`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH - 1:0];
    assign read_search_data_current = ram_read_data != 0 && read_search_data_time_diff < `FLOW_TIMESTAMP_TIMEOUT;
    always @(posedge clk) begin
        if (reset) begin
            // Reset
            read_search_counter_res <= 0;
            read_search_res_new <= 1;
        end else begin
            if (ram_read_data_new) begin
                // Counter of received results
                if (read_search_counter_res == `FLOW_RAM_HASHES-1) begin
                    read_search_res_new <= 1;
                    read_search_counter_res <= 0;
                end else begin
                    read_search_res_new <= 0;
                    read_search_counter_res <= read_search_counter_res + 1;
                end
                // Check the read value and update the result if needed
                if (read_search_data_current && read_search_data_hash_ok) begin
                    // The hash is good
                    read_search_result <= ram_read_data[`FLOW_DATA_WIDTH-1:0];
                    read_search_found <= 1;
                end else if (read_search_res_new) begin
                    read_search_found <= 0;
                    read_search_result <= 0;
                end
                // Remember the hash with the highest redundancy
                if (read_search_res_new || read_search_data_redundancy > read_search_highest_redundancy) begin
                    read_search_highest_redundancy <= read_search_data_redundancy;
                    read_search_highest_redundancy_hash <= read_search_counter_res;
                end
            end
            // Tell the send step when it should send
            if (ram_read_data_new && read_search_counter_res == `FLOW_RAM_HASHES-1)
                read_send_enable <= 1;
            else
                read_send_enable <= 0;
        end
    end
    // Remember hashes which can be written
    generate
        for (hash_id = 0; hash_id < `FLOW_RAM_HASHES; hash_id = hash_id + 1) begin: hash_active
            always @(posedge clk) begin
                if (ram_read_data_new) begin
                if (hash_id == read_search_counter_res)
                    read_search_active_hashes[hash_id] <= !read_search_data_current || read_search_data_hash_ok;
                else if (read_search_res_new)
                    read_search_active_hashes[hash_id] <= 0;
                end
            end
        end
    endgenerate

    // Send step: manage the output
    assign read_edit_id = {read_active_hashes, read_hashes_merged};
    always @(posedge clk) begin
        if (read_send_enable) begin
            // Data found?
            read_data_found <= read_search_found;
            // Data if found
            read_data <= read_search_result;
            // Active hashes bit map
            if (read_search_active_hashes != 0) begin
                // Some empty cells found
                read_active_hashes <= read_search_active_hashes;
            end else begin
                read_active_hashes <= 1 << read_search_highest_redundancy_hash;
            end
            // Hashes management
            read_hashes_merged <= read_hashes_fifo_out;
        end
        // Management of the "new" signal
        read_data_new <= read_send_enable;
    end

    // Write FSM
    always @(posedge clk) begin
        if (reset)
            write_state <= WRITE_RESET;
        else
            write_state <= write_state_nxt;
    end

    always @(*) begin
        write_state_nxt = write_state;
        write_ready = 0;
        write_save_input = 0;
        write_counter_en = 0;
        write_counter_reset = 0;
        write_ram_en = 0;

        case (write_state)
            WRITE_RESET: begin
                write_state_nxt = WRITE_READY;
            end
            WRITE_READY: begin
                if (write_en) begin
                    if (ram_write_ready)
                        write_state_nxt = WRITE_WRITE;
                    else
                        write_state_nxt = WRITE_WAIT;
                    write_save_input = 1;
                    write_counter_reset = 1;
                end else begin
                    write_ready = 1;
                end
            end
            WRITE_WAIT: begin
                if (ram_write_ready)
                    write_state_nxt = WRITE_WRITE;
            end
            WRITE_WRITE: begin
                write_counter_en = 1;
                write_ram_en = 1;
                if (write_counter == 0) begin
                    write_state_nxt = WRITE_READY;
                    write_ready = 1;
                end else if (!ram_write_ready) begin
                    write_state_nxt = WRITE_WAIT;
                end
            end
        endcase
    end

    // Input saving management
    assign write_hashes = write_edit_id[`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES-1:0];
    assign write_active_hashes = write_edit_id[`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES + `FLOW_RAM_HASHES-1:`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES];
    generate
        for (hash_id = 0; hash_id < `FLOW_RAM_HASHES; hash_id = hash_id + 1) begin: hash_save
            // Hashes
            always @(posedge clk) begin
                if (write_save_input)
                    write_hash_saved[hash_id] <= write_hashes[(hash_id+1) * `FLOW_RAM_FULL_HASH_WIDTH - 1:hash_id * `FLOW_RAM_FULL_HASH_WIDTH];
            end
        end
    endgenerate
    always @(posedge clk) begin
        if (write_save_input) begin
            write_hash_active_saved <= write_active_hashes;
            write_data_saved <= write_data;
        end
    end

    // Write counter
    // Inverted to save in the last address first:
    // best chances to read an up-to-date value if reading and
    // writing of the same address are concurrent.
    always @(posedge clk) begin
        if (write_counter_reset)
            write_counter <= `FLOW_RAM_HASHES - 1;
        else if (write_counter_en)
            write_counter <= write_counter - 1;
    end

    // RAM input write data management
    assign ram_write_en = write_ram_en && write_hash_active_saved[write_counter];
    assign sum_bits_in = write_hash_active_saved;
    assign write_redundancy = sum_bits_out - 1;
    assign write_hash_saved_cur = write_hash_saved[write_counter];
    // Recreate the same ALT part as during read
    assign ram_write_addr = {write_counter[`FLOW_RAM_ADDR_ALT_WIDTH-1:0], write_hash_saved_cur[`FLOW_RAM_FULL_HASH_WIDTH-1:`FLOW_RAM_FULL_HASH_WIDTH-`FLOW_RAM_ADDR_RAND_WIDTH]};
    assign ram_write_data = {write_hash_saved_cur[`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH - 1:0], timestamp, write_redundancy, write_data_saved};

endmodule
