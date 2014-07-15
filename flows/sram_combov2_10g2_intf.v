// Interface to the Combov2 RAM made to simplify its use.
// The interface is adapted to flow data storage.
// There are read and write interfaces, which may be used only when ready.
// The read interface returns the data after n clock cycles and
//   sets read_data_new to 1.
// Two memories are used in parallel, using the first bit of the address

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"

`define SRAM_COMBOV2_CONCURRENT_REQUESTS_BITS 6

module sram_combov2_10g2_intf(
    // Simplified interface
    input clk,
    input reset,
    // Write
    output reg write_ready,
    input write_en,
    input [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr,
    input [`FLOW_RAM_WORD_WIDTH-1:0] write_data,
    // Read
    output reg read_ready,
    input read_en,
    input [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr,
    output [`FLOW_RAM_WORD_WIDTH-1:0] read_data,
    output read_data_new,

    // Interface to Combov2 SRAM
    // Read 0
    output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_0_rd_addr,
    output                                qdr_0_rd_req,
    input                                 qdr_0_rd_rdy,
    input [`FLOW_RAM_WORD_WIDTH-1:0]      qdr_0_rd_data,
    input                                 qdr_0_rd_dv,
    // Write 0
    output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_0_wr_addr,
    output                                qdr_0_wr_req,
    input                                 qdr_0_wr_rdy,
    output [`FLOW_RAM_WORD_WIDTH-1:0]     qdr_0_wr_data,
    // Read 1
    output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_1_rd_addr,
    output                                qdr_1_rd_req,
    input                                 qdr_1_rd_rdy,
    input [`FLOW_RAM_WORD_WIDTH-1:0]      qdr_1_rd_data,
    input                                 qdr_1_rd_dv,
    // Write 1
    output [`FLOW_RAM_ADDR_WIDTH-2:0]     qdr_1_wr_addr,
    output                                qdr_1_wr_req,
    input                                 qdr_1_wr_rdy,
    output [`FLOW_RAM_WORD_WIDTH-1:0]     qdr_1_wr_data
    );

    // Declarations

    // Read temporary storage
    reg [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr_reg;
    reg read_store_input;

    // Write temporary storage
    reg [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr_reg;
    reg [`FLOW_RAM_WORD_WIDTH-1:0] write_data_reg;
    reg write_store_input;

    // Read control
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr_ram_full;
    wire [`FLOW_RAM_ADDR_WIDTH-2:0] read_addr_ram;
    wire read_ram_index;
    wire read_ram_ready;
    reg read_req_ram;

    // Write control
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr_ram_full;
    wire [`FLOW_RAM_ADDR_WIDTH-2:0] write_addr_ram;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] write_data_ram;
    wire write_ram_index;
    wire write_ram_ready;
    reg write_req_ram;

    // FSMs
    reg [1:0] read_state, read_state_nxt;
    localparam READ_RESET = 0;
    localparam READ_READY = 1;
    localparam READ_SEND = 2;
    reg [1:0] write_state, write_state_nxt;
    localparam WRITE_RESET = 0;
    localparam WRITE_READY = 1;
    localparam WRITE_SEND = 2;

    // FIFOs to manage the read output of two RAMs
    wire fifo_read_req_full;
    wire fifo_ram0_read;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] fifo_ram0_out;
    wire fifo_ram0_empty;
    wire fifo_ram1_read;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] fifo_ram1_out;
    wire fifo_ram1_empty;

	// Fifo to store the order of the requests to RAMs
    // Requires 1 clock cycle: the RAM should too! (and does)
	fallthrough_small_fifo
	#(
		.WIDTH(1),
		.MAX_DEPTH_BITS(`SRAM_COMBOV2_CONCURRENT_REQUESTS_BITS)
	)
	fifo_read_req
   (	.din           (read_ram_index),
		.wr_en         (read_req_ram),
		.rd_en         (read_data_new),
		.dout          (read_ram_index_out),
		.full          (fifo_read_req_full),
		.reset         (reset),
		.clk           (clk)
	);

	// Fifo to store the output of RAM 0
    // No overflow because it can store all concurrent requests
	fallthrough_fast_fifo
	#(
		.WIDTH(`FLOW_RAM_WORD_WIDTH),
		.MAX_DEPTH_BITS(`SRAM_COMBOV2_CONCURRENT_REQUESTS_BITS)
	)
	fifo_ram0
   (	.din           (qdr_0_rd_data),
		.wr_en         (qdr_0_rd_dv),
		.rd_en         (fifo_ram0_read),
		.dout          (fifo_ram0_out),
		.empty         (fifo_ram0_empty),
		.reset         (reset),
		.clk           (clk)
	);

	// Fifo to store the output of RAM 1
    // No overflow because it can store all concurrent requests
	fallthrough_fast_fifo
	#(
		.WIDTH(`FLOW_RAM_WORD_WIDTH),
		.MAX_DEPTH_BITS(`SRAM_COMBOV2_CONCURRENT_REQUESTS_BITS)
	)
	fifo_ram1
   (	.din           (qdr_1_rd_data),
		.wr_en         (qdr_1_rd_dv),
		.rd_en         (fifo_ram1_read),
		.dout          (fifo_ram1_out),
		.empty         (fifo_ram1_empty),
		.reset         (reset),
		.clk           (clk)
	);

    // Logic

    // READ FSM
    always @(posedge clk) begin
        if (reset)
            read_state <= READ_RESET;
        else
            read_state <= read_state_nxt;
    end
    always @(*) begin
        read_state_nxt = read_state;
        read_ready = 0;
        read_store_input = 0;
        read_req_ram = 0;
        case (read_state)
            READ_RESET: begin
                read_state_nxt = READ_READY;
            end
            READ_READY: begin
                if (read_en) begin
                    read_store_input = 1;
                    if (!read_ram_ready || fifo_read_req_full) begin
                        read_state_nxt = READ_SEND;
                    end else begin
                        read_ready = 1;
                        read_req_ram = 1;
                    end
                end else begin
                    read_ready = 1;
                end
            end
            READ_SEND: begin
                if (read_ram_ready && !fifo_read_req_full) begin
                    read_req_ram = 1;
                    read_state_nxt = READ_READY;
                    read_ready = 1;
                end
            end
        endcase
    end

    // Read SRAM connection
    assign read_addr_ram_full = read_store_input ? read_addr : read_addr_reg;
    assign read_addr_ram = read_addr_ram_full[`FLOW_RAM_ADDR_WIDTH-2:0];
    assign read_ram_index = read_addr_ram_full[`FLOW_RAM_ADDR_WIDTH-1];
    assign read_ram_ready = (read_ram_index == 0 && qdr_0_rd_rdy) || (read_ram_index == 1 && qdr_1_rd_rdy);
    assign qdr_0_rd_addr = read_addr_ram;
    assign qdr_1_rd_addr = read_addr_ram;
    assign qdr_0_rd_req = read_req_ram && read_ram_index == 0;
    assign qdr_1_rd_req = read_req_ram && read_ram_index == 1;
    always @(posedge clk) begin
        if (read_store_input)
            read_addr_reg <= read_addr;
    end

    // Read FIFO management (read output)
    assign read_data_new = fifo_ram0_read || fifo_ram1_read;
    assign read_data = read_ram_index_out ? fifo_ram1_out : fifo_ram0_out;
    assign fifo_ram0_read = read_ram_index_out == 0 && !fifo_ram0_empty;
    assign fifo_ram1_read = read_ram_index_out == 1 && !fifo_ram1_empty;

    // WRITE FSM
    always @(posedge clk) begin
        if (reset)
            write_state <= WRITE_RESET;
        else
            write_state <= write_state_nxt;
    end
    always @(*) begin
        write_state_nxt = write_state;
        write_ready = 0;
        write_req_ram = 0;
        write_store_input = 0;

        case (write_state)
            WRITE_RESET: begin
                write_state_nxt = WRITE_READY;
            end
            WRITE_READY: begin
                if (write_en) begin
                    write_store_input = 1;
                    if (!write_ram_ready) begin
                        write_state_nxt = WRITE_SEND;
                    end else begin
                        write_req_ram = 1;
                        write_ready = 1;
                    end
                end else begin
                    write_ready = 1;
                end
            end
            WRITE_SEND: begin
                if (write_ram_ready) begin
                    write_ready = 1;
                    write_req_ram = 1;
                    write_state_nxt = WRITE_READY;
                end
            end
        endcase
    end

    // Write SRAM connections
    assign write_addr_ram_full = write_store_input ? write_addr : write_addr_reg;
    assign write_addr_ram = write_addr_ram_full[`FLOW_RAM_ADDR_WIDTH-2:0];
    assign write_ram_index = write_addr_ram_full[`FLOW_RAM_ADDR_WIDTH-1];
    assign write_data_ram = write_store_input ? write_data : write_data_reg;
    assign write_ram_ready = (write_ram_index == 0 && qdr_0_wr_rdy) || (write_ram_index == 1 && qdr_1_wr_rdy);
    assign qdr_0_wr_addr = write_addr_ram;
    assign qdr_1_wr_addr = write_addr_ram;
    assign qdr_0_wr_data = write_data_ram;
    assign qdr_1_wr_data = write_data_ram;
    assign qdr_0_wr_req = write_req_ram && write_ram_index == 0;
    assign qdr_1_wr_req = write_req_ram && write_ram_index == 1;
    always @(posedge clk) begin
        if (write_store_input) begin
            write_data_reg <= write_data;
            write_addr_reg <= write_addr;
        end
    end

endmodule
