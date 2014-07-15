//Fallthrough FIFO:
// the oldest stored value is always available at the FIFO output.
// Setting read_en signals to read the next value.
// Data goes through as fast as possible:
// no delay if empty

`timescale 1ns / 1ps

module fallthrough_fast_fifo
#(
    parameter WIDTH = 72,
    parameter MAX_DEPTH_BITS = 3
)
(    
    input [WIDTH-1:0]       din,     // Data in
    input                   wr_en,   // Write enable
     
    input                   rd_en,   // Read the next word 
    output reg [WIDTH-1:0]  dout,    // Data out
     
    output reg              full,
    output reg              nearly_full, // Only 1 word left
    output reg              full_nxt, // Will be full at next cycle
    output reg              empty,
    
    input                   reset,
    input                   clk
);

    // FIFO connections
    wire [WIDTH-1:0] fifo_din;
    wire [WIDTH-1:0] fifo_dout;
    reg fifo_wr_en;
    reg fifo_rd_en;
    wire fifo_empty;
    wire fifo_full;
    wire fifo_nearly_full;
    
    // Data storage
    reg [WIDTH-1:0] data_reg;
    reg data_keep;
    
    // FSM signals
    localparam OUT_EMPTY = 0;
    localparam OUT_KEEP = 1;
    localparam OUT_FIFO = 2;
    reg [1:0] state, state_nxt;

    // FIFO instantiation
    small_fifo
    #(
        .WIDTH (WIDTH),
        .MAX_DEPTH_BITS (MAX_DEPTH_BITS)
    ) fifo (
        .din           (fifo_din),
        .wr_en         (fifo_wr_en),
        .rd_en         (fifo_rd_en),
        .dout          (fifo_dout),
        .full          (fifo_full),
        .nearly_full   (fifo_nearly_full),
        .empty         (fifo_empty),
        .reset         (reset),
        .clk           (clk)
    );

    // Direct connections
    assign fifo_din = din;
    
    // Register that keeps the latest value
    always @(posedge clk) begin
        if (data_keep)
            data_reg <= din;
    end
    
    // FSM: ensure that there is always a result at FIFO output
    always @(posedge clk) begin
        if (reset)
            state <= OUT_EMPTY;
        else
            state <= state_nxt;
    end
    always @(*) begin
        state_nxt = state;
        data_keep = 0;
        fifo_rd_en = 0;
        fifo_wr_en = 0;
        empty = 1;
        full = 1;
        full_nxt = 1;
        nearly_full = 1;
        dout = din;
        case (state)
            OUT_EMPTY: begin
                if (wr_en) begin
                    if (!rd_en) begin
                        data_keep = 1;
                        state_nxt = OUT_KEEP;
                        full_nxt = fifo_full;
                    end else begin
                        full_nxt = 0;                        
                    end
                    empty = 0;
                end else begin
                    empty = 1;
                    full_nxt = 0;
                end
                full = 0;
                nearly_full = fifo_full;
                dout = din;
            end
            OUT_KEEP: begin
                if (rd_en) begin
                    if (fifo_empty) begin
                        if (wr_en) begin
                            data_keep = 1;
                            full_nxt = fifo_full;
                        end else begin
                            state_nxt = OUT_EMPTY;
                            full_nxt = 0;
                        end
                    end else begin
                        state_nxt = OUT_FIFO;
                        fifo_wr_en = wr_en;
                        fifo_rd_en = 1;
                        full_nxt = fifo_full && wr_en;
                    end
                end else begin
                    fifo_wr_en = wr_en;
                    full_nxt = fifo_full || (wr_en && fifo_nearly_full);
                end
                empty = 0;
                full = fifo_full;
                nearly_full = fifo_nearly_full;
                dout = data_reg;
            end
            OUT_FIFO: begin
                if (rd_en) begin
                    if (fifo_empty) begin
                        if (wr_en) begin
                            data_keep = 1;
                            state_nxt = OUT_KEEP;
                            full_nxt = fifo_full;
                        end else begin
                            state_nxt = OUT_EMPTY;
                            full_nxt = fifo_full && wr_en;
                        end
                    end else begin
                        fifo_wr_en = wr_en;
                        fifo_rd_en = 1;
                        full_nxt = 0;
                    end
                end else begin
                    fifo_wr_en = wr_en;
                    full_nxt = fifo_full || (wr_en && fifo_nearly_full);
                end
                empty = 0;
                full = fifo_full;
                nearly_full = fifo_nearly_full;
                dout = fifo_dout;
            end
        endcase
    end
endmodule
