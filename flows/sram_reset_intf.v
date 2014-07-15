// This encapsulates an SRAM interface
// and adds reset management by using the first clock cycles to reset.
// Adds simultaneous read/write management too so that the written value is returned if it is at the
// same address and clock cycle as the read value
// Read output is registered: it adds one clock cycle but cuts the critical path

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"

module sram_reset_intf(
    // Simplified interface
    input clk,
    input reset,
    // Write
    output write_ready,
    input write_en,
    input [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr,
    input [`FLOW_RAM_WORD_WIDTH-1:0] write_data,
    // Read
    output read_ready,
    input read_en,
    input [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr,
    output reg [`FLOW_RAM_WORD_WIDTH-1:0] read_data,
    output reg read_data_new
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

    // SRAM adapter connection
    wire write_ready_in;
    wire read_ready_in;
    wire write_en_in;
    wire read_en_in;
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr_in;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] write_data_in;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] read_data_in;
    wire read_data_new_in;

    `ifdef NETFPGA_1G
        // NetFPGA 1G adapter
        sram_netfpga1g_intf ram
        (
            .clk(clk),
            .reset(reset),

            .write_ready(write_ready_in),
            .write_en(write_en_in),
            .write_addr(write_addr_in),
            .write_data(write_data_in),

            .read_ready(read_ready_in),
            .read_en(read_en_in),
            .read_addr(read_addr),
            .read_data(read_data_in),
            .read_data_new(read_data_new_in),

            .wr_0_addr          (wr_0_addr),
            .wr_0_req           (wr_0_req),
            .wr_0_ack           (wr_0_ack),
            .wr_0_data          (wr_0_data),
            .rd_0_ack           (rd_0_ack),
            .rd_0_data          (rd_0_data),
            .rd_0_vld           (rd_0_vld),
            .rd_0_addr          (rd_0_addr),
            .rd_0_req           (rd_0_req)
        );
    `else `ifdef COMBOV2_10G2
        // Combov2 RAM adapter
        sram_combov2_10g2_intf ram
        (
            .clk(clk),
            .reset(reset),

            .write_ready(write_ready_in),
            .write_en(write_en_in),
            .write_addr(write_addr_in),
            .write_data(write_data_in),

            .read_ready(read_ready_in),
            .read_en(read_en_in),
            .read_addr(read_addr),
            .read_data(read_data_in),
            .read_data_new(read_data_new_in),

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
        );
    `else
        // Block RAM adapter
        sram_intf ram
        (
            .clk(clk),
            .reset(reset),

            .write_ready(write_ready_in),
            .write_en(write_en_in),
            .write_addr(write_addr_in),
            .write_data(write_data_in),

            .read_ready(read_ready_in),
            .read_en(read_en_in),
            .read_addr(read_addr),
            .read_data(read_data_in),
            .read_data_new(read_data_new_in)
        );
    `endif `endif

    // FIFO connection (to manage simultaneous data read and write)
    reg write_en_reg;
    reg [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr_reg, write_addr_reg;
    reg [`FLOW_RAM_WORD_WIDTH-1:0] write_data_reg;
    wire fifo_simult_full, simult_read_write;
    wire [`FLOW_RAM_WORD_WIDTH-1:0] read_data_simult;

    fallthrough_small_fifo
    #(
        .WIDTH(`FLOW_RAM_WORD_WIDTH+1),
        .MAX_DEPTH_BITS(`FLOW_RAM_LATENCY_BITS)
    )
    fifo_simult
   (    .din           ({write_en_reg && (read_addr_reg == write_addr_reg), write_data_reg}),
        .wr_en         (read_en),
        .rd_en         (read_data_new),
        .dout          ({simult_read_write, read_data_simult}),
        .nearly_full   (fifo_simult_full),
        .reset         (reset),
        .clk           (clk)
    );

    // Internal variables
    reg in_reset, in_reset_n, finished_reset;
    reg write_reset;
    wire [`FLOW_RAM_ADDR_WIDTH-1:0] addr_counter;
    reg [`FLOW_RAM_ADDR_ALT_WIDTH-1:0] addr_counter_alt;
    reg [`FLOW_RAM_ADDR_RAND_WIDTH-1:0] addr_counter_rand;
    reg addr_counter_en;
    reg [1:0] state, state_nxt;
    reg [`FLOW_RAM_WORD_WIDTH-1:0] read_data_synth;
    localparam STATE_RESET = 0;
    localparam STATE_WAIT = 1;
    localparam STATE_WRITE = 2;
    localparam STATE_DONE = 3;

    // Logic

    // RAM management
    assign write_ready = write_ready_in && in_reset_n;
    assign read_ready = read_ready_in && !fifo_simult_full && in_reset_n;
    assign write_en_in = (write_en && in_reset_n) || (write_reset && in_reset);
    assign read_en_in = read_en && in_reset_n;
    assign write_addr_in = in_reset ? addr_counter : write_addr;
    assign write_data_in = in_reset ? {(`FLOW_RAM_WORD_WIDTH){1'b0}} : write_data;
    always @(posedge clk) begin
        read_data_synth <= simult_read_write ? read_data_simult : read_data_in;
        read_data_new <= read_data_new_in;
    end
    // In simulation only: remove X values from output (no time to initialize the RAM)
    // => could hide bugs, be careful!
    always @(*) begin
        read_data = read_data_synth;
        // synthesis translate_off
        if (reset || read_data_synth === {(`FLOW_RAM_WORD_WIDTH){1'bx}}) begin
            read_data = {(`FLOW_RAM_WORD_WIDTH){1'b0}};
        end
        // synthesis translate_on
    end

    // Register all input for simultaneous input check
    always @(posedge clk) begin
        write_en_reg <= write_en;
        read_addr_reg <= read_addr;
        write_addr_reg <= write_addr;
        write_data_reg <= write_data;
    end

    // Address counter, makes the "alt" part of the address vary each clock cycle
    assign addr_counter = {addr_counter_alt, addr_counter_rand};
    always @(posedge clk) begin
        if (reset) begin
            addr_counter_alt <= 0;
            addr_counter_rand <= 0;
        end else if (addr_counter_en) begin
            addr_counter_alt <= addr_counter_alt + 1;
            if (addr_counter_alt == {(`FLOW_RAM_ADDR_ALT_WIDTH){1'b1}})
                addr_counter_rand <= addr_counter_rand + 1;
        end
    end

    // Signal of the reset phase
    always @(posedge clk) begin
        if (reset) begin
            in_reset <= 1;
            in_reset_n <= 0;
        end else if (finished_reset) begin
            in_reset <= 0;
            in_reset_n <= 1;
        end
    end

    // FSM
    always @(posedge clk) begin
        if (reset)
            state <= STATE_RESET;
        else
            state <= state_nxt;
    end

    always @(*) begin
        state_nxt = state;
        finished_reset = 0;
        write_reset = 0;
        addr_counter_en = 0;

        case (state)
            STATE_RESET: begin
                state_nxt = STATE_WAIT;
                // Skips the reset in simulation only
                // synthesis translate_off
                state_nxt = STATE_DONE;
                // synthesis translate_on
            end

            STATE_WAIT: begin
                if (write_ready_in)
                    state_nxt = STATE_WRITE;
            end

            STATE_WRITE: begin
                write_reset = 1;
                addr_counter_en = 1;
                if (addr_counter == {(`FLOW_RAM_ADDR_WIDTH){1'b1}})
                    state_nxt = STATE_DONE;
                else if (write_ready_in)
                    state_nxt = STATE_WRITE;
                else
                    state_nxt = STATE_WAIT;
            end

            STATE_DONE: begin
                // Nothing to do: module done
                finished_reset = 1;
            end
        endcase
    end
endmodule
