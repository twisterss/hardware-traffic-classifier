// Interface to the SRAM made to simplify its use.
// The interface is adapted to flow data storage.
// There are read and write interfaces, which may be used only when ready.
// The read interface returns the data after n clock cycles and
//   sets read_data_valid to 1.
// FOR NOW, it is not connected to a SRAM but to synthetized RAM.
// The use of 2 SRAMs with a latency of 2 clock cycles is simulated:
// the first bit of the address is the block id, if 2 read requests
// are received consecutively on the same block, it induces a 1-clock cycle delay.
// The same is true for write requests.

`timescale 1ns / 1ps

`include "config/traffic_parameters.v"
`define SRAM_DELAY 12

module sram_intf(
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
    output read_data_new
    );

    // Decalarations

    // 2-cycles latency RAM simulation
    reg [`FLOW_RAM_WORD_WIDTH-1:0] ram [0:(1<<`FLOW_RAM_ADDR_WIDTH)-1];
    reg ram_read;
    reg ram_read_delayed;
    reg [`FLOW_RAM_ADDR_WIDTH-1:0] ram_read_addr;
    reg ram_write;
    reg ram_write_delayed;
    reg [`FLOW_RAM_ADDR_WIDTH-1:0] ram_write_addr;
    reg [`FLOW_RAM_WORD_WIDTH-1:0] ram_write_data;
    // Temporary storage to simulate lower bandwidth (same block read or written)
    reg ram_store_read;
    reg ram_store_write;
    // Current and previous blocks
    wire [`FLOW_RAM_ADDR_ALT_WIDTH-1:0] read_block;
    wire [`FLOW_RAM_ADDR_ALT_WIDTH-1:0] write_block;
    reg [`FLOW_RAM_ADDR_ALT_WIDTH:0] read_block_prev;
    reg [`FLOW_RAM_ADDR_ALT_WIDTH:0] write_block_prev;
    // Output delaying
    reg [`FLOW_RAM_WORD_WIDTH-1:0] read_data_reg [0:`SRAM_DELAY-1];
    reg read_data_new_reg [0:`SRAM_DELAY-1];
    // FSMs
    reg [1:0] read_state, read_state_nxt;
    localparam READ_RESET = 0;
    localparam READ_READY = 1;
    localparam READ_WAITING = 2;
    reg [1:0] write_state, write_state_nxt;
    localparam WRITE_RESET = 0;
    localparam WRITE_READY = 1;
    localparam WRITE_WAITING = 2;


    // Simulation initialization
    // synthesis translate_off
    integer k;
    initial begin
        for (k = 0; k < (1<<`FLOW_RAM_ADDR_WIDTH); k=k+1)
            ram[k] = {(`FLOW_RAM_WORD_WIDTH){1'b0}};
    end
    // synthesis translate_on

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
        ram_read = 0;
        ram_store_read = 0;

        case (read_state)
            READ_RESET: begin
                read_state_nxt = READ_READY;
            end
            READ_READY: begin
                if (read_en) begin
                    ram_store_read = 1;
                    if (read_block_prev != read_block) begin
                        ram_read = 1;
                        read_ready = 1;
                    end else begin
                        read_state_nxt = READ_WAITING;
                    end
                end else begin
                    read_ready = 1;
                end
            end
            READ_WAITING: begin
                ram_read = 1;
                read_ready = 1;
                read_state_nxt = READ_READY;
            end
        endcase
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
        ram_write = 0;
        ram_store_write = 0;

        case (write_state)
            WRITE_RESET: begin
                write_state_nxt = WRITE_READY;
            end
            WRITE_READY: begin
                if (write_en) begin
                    ram_store_write = 1;
                    if (write_block_prev != write_block) begin
                        ram_write = 1;
                        write_ready = 1;
                    end else begin
                        write_state_nxt = WRITE_WAITING;
                    end
                end else begin
                    write_ready = 1;
                end
            end
            WRITE_WAITING: begin
                ram_write = 1;
                write_ready = 1;
                write_state_nxt = WRITE_READY;
            end
        endcase
    end

    // Store the block id on the last clock cycle, if the RAM was active (for read and write)
    assign read_block = read_addr[`FLOW_RAM_ADDR_WIDTH-1:`FLOW_RAM_ADDR_WIDTH-`FLOW_RAM_ADDR_ALT_WIDTH];
    assign write_block = write_addr[`FLOW_RAM_ADDR_WIDTH-1:`FLOW_RAM_ADDR_WIDTH-`FLOW_RAM_ADDR_ALT_WIDTH];
    always @(posedge clk) begin
        read_block_prev <= ram_store_read ? read_block : ram_read ? read_block_prev : -1;
        write_block_prev <= ram_store_write ? write_block : ram_write ? write_block_prev : -1;
    end

    // Delayed RAM inputs
    always @(posedge clk) begin
        ram_read_delayed <= ram_read;
        ram_write_delayed <= ram_write;
        if (ram_store_read)
            ram_read_addr <= read_addr;
        if (ram_store_write) begin
            ram_write_addr <= write_addr;
            ram_write_data <= write_data;
        end
    end

    // Delayed RAM output
    assign read_data_new = read_data_new_reg[`SRAM_DELAY-1];
    assign read_data = read_data_reg[`SRAM_DELAY-1];
    generate
        genvar i;
        for (i = 1; i < `SRAM_DELAY; i = i + 1) begin: ram_delayer
            always @(posedge clk) begin
                if (reset) begin
                    read_data_new_reg[i] <= 0;
                end else begin
                    read_data_reg[i] <= read_data_reg[i-1];
                    read_data_new_reg[i] <= read_data_new_reg[i-1];
                end
            end
        end
    endgenerate

    // RAM mechanism
    always @(posedge clk) begin
        if (reset) begin
            read_data_new_reg[0] <= 0;
        end else begin
            if (ram_read_delayed) begin
                read_data_reg[0] <= ram[ram_read_addr];
                read_data_new_reg[0] <= 1;
            end else begin
                read_data_new_reg[0] <= 0;
            end
            if (ram_write_delayed) begin
                ram[ram_write_addr] <= ram_write_data;
            end
        end
    end
endmodule
