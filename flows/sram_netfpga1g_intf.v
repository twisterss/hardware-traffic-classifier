// Interface to the NetFPGA SRAM made to simplify its use.
// The interface is adapted to flow data storage.
// There are read and write interfaces, which may be used only when ready.
// The read interface returns the data after n clock cycles and
//   sets read_data_valid to 1.

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/traffic_parameters.v"

module sram_netfpga1g_intf(
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
    
    // Interface to NetFPGA SRAM
    // Write
    output [`FLOW_RAM_ADDR_WIDTH-1:0]     wr_0_addr,
    output reg                            wr_0_req,
    input                                 wr_0_ack,
    output [`FLOW_RAM_WORD_WIDTH-1:0]     wr_0_data,
    // Read
    input                                 rd_0_ack,
    input  [`FLOW_RAM_WORD_WIDTH-1:0]     rd_0_data,
    input                                 rd_0_vld,
    output [`FLOW_RAM_ADDR_WIDTH-1:0]     rd_0_addr,
    output reg                            rd_0_req
    );
    
    // Declarations
    
    // Read temporary storage
    reg [`FLOW_RAM_ADDR_WIDTH:0] read_addr_reg;
    reg read_store_input;
    
    // Write temporary storage
    reg [`FLOW_RAM_ADDR_WIDTH:0] write_addr_reg;
    reg [`FLOW_RAM_WORD_WIDTH-1:0] write_data_reg;
    reg write_store_input;
    
    // FSMs
    reg [1:0] read_state, read_state_nxt;
    localparam READ_RESET = 0;
    localparam READ_READY = 1;
    localparam READ_SEND = 2;
    reg [1:0] write_state, write_state_nxt;
    localparam WRITE_RESET = 0;
    localparam WRITE_READY = 1;
    localparam WRITE_SEND = 2;
    
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
        rd_0_req = 0;
        case (read_state)
            READ_RESET: begin
                read_state_nxt = READ_READY;
            end
            READ_READY: begin
                if (read_en) begin
                    rd_0_req = 1;
                    read_store_input = 1;
                    if (!rd_0_ack)
                        read_state_nxt = READ_SEND;
                    else
                        read_ready = 1;
                end else begin
                    read_ready = 1;
                end                
            end
            READ_SEND: begin
                rd_0_req = 1;
                if (rd_0_ack) begin
                    read_state_nxt = READ_READY;
                    read_ready = 1;
                end
            end
        endcase
    end
    
    // Read SRAM connection
    assign rd_0_addr = read_store_input ? read_addr : read_addr_reg;
    assign read_data_new = rd_0_vld;
    assign read_data = rd_0_data;
    always @(posedge clk) begin
        if (read_store_input)
            read_addr_reg <= read_addr;
    end
    
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
        wr_0_req = 0;
        write_store_input = 0;
        
        case (write_state)
            WRITE_RESET: begin
                write_state_nxt = WRITE_READY;
            end
            WRITE_READY: begin
                if (write_en) begin
                    wr_0_req = 1;
                    write_store_input = 1;
                    if (!wr_0_ack)
                        write_state_nxt = WRITE_SEND;
                    else
                        write_ready = 1;
                end else begin
                    write_ready = 1;
                end
            end
            WRITE_SEND: begin
                wr_0_req = 1;
                if (wr_0_ack) begin
                    write_ready = 1;
                    write_state_nxt = WRITE_READY;
                end
            end
        endcase
    end
    
    // Write SRAM connections
    assign wr_0_addr = write_store_input ? write_addr : write_addr_reg;
    assign wr_0_data = write_store_input ? write_data : write_data_reg;
    always @(posedge clk) begin
        if (write_store_input) begin
            write_data_reg <= write_data;
            write_addr_reg <= write_addr;
        end
    end
    
endmodule
