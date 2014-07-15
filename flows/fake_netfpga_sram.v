// Fake NetFPGA SRAM used for simulation
// The interface is the one from the NetFPGA controller

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"

module fake_netfpga_sram(
    input                                 clk,
    input                                 reset,
    // Write
    input [`FLOW_RAM_ADDR_WIDTH-1:0]      wr_0_addr,
    input                                 wr_0_req,
    input [`FLOW_RAM_WORD_WIDTH-1:0]      wr_0_data,
    output reg                            wr_0_ack,
    // Read
    output reg                            rd_0_ack,
    output reg [`FLOW_RAM_WORD_WIDTH-1:0] rd_0_data,
    output reg                            rd_0_vld,
    input [`FLOW_RAM_ADDR_WIDTH-1:0]      rd_0_addr,
    input                                 rd_0_req
    );

    // Dual-port BlockRAM logic
    reg [`FLOW_RAM_WORD_WIDTH-1:0] ram [0:(1<<`FLOW_RAM_ADDR_WIDTH)-1];
    always @(posedge clk) begin
        if (rd_0_req && rd_0_ack)
            rd_0_data <= ram[rd_0_addr];
        if (wr_0_req && wr_0_ack)
            ram[wr_0_addr] <= wr_0_data;
    end
    
    // Counter to delay responses
    reg [2:0] counter;
    always @(posedge clk) begin
        if (reset)
            counter <= 0;
        else
            counter <= counter + 1;
    end
    
    // Read control FSM
    reg read_state, read_state_nxt;
    localparam READ_READY = 0;
    localparam READ_SEND = 1;
    always @(posedge clk) begin
        if (reset)
            read_state <= READ_READY;
        else
            read_state <= read_state_nxt;
    end
    always @(*) begin
        read_state_nxt = read_state;
        rd_0_ack = 0;
        rd_0_vld = 0;
        case (read_state)
            READ_READY: begin
                if (rd_0_req && counter == 0) begin
                    rd_0_ack = 1;
                    read_state_nxt = READ_SEND;
                end
            end
            READ_SEND: begin
                if (counter == 2) begin
                    rd_0_vld = 1;
                    read_state_nxt = READ_READY;
                end
            end
        endcase
    end
    
    // Write control FSM
    reg write_state, write_state_nxt;
    localparam WRITE_READY = 0;
    localparam WRITE_SEND = 1;
    always @(posedge clk) begin
        if (reset)
            write_state <= WRITE_READY;
        else
            write_state <= write_state_nxt;
    end
    always @(*) begin
        write_state_nxt = write_state;
        wr_0_ack = 0;
        case (write_state)
            WRITE_READY: begin
                if (wr_0_req && counter == 3) begin
                    wr_0_ack = 1;
                    write_state_nxt = WRITE_SEND;
                end
            end
            WRITE_SEND: begin
                if (counter == 0) begin
                    write_state_nxt = WRITE_READY;
                end
            end
        endcase
    end
endmodule
