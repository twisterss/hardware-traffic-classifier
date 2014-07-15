`timescale 1ns / 1ps
// Timestamp generator
// Generate a timestamp of width TIMESTAMP_WIDTH.
// Time unit it TIMESTAMP_UNIT * clock period
module timestamp_gen #(
    parameter TIMESTAMP_WIDTH = 5,
    parameter TIMESTAMP_UNIT_WIDTH = 32,
    parameter TIMESTAMP_UNIT = 1 << TIMESTAMP_UNIT_WIDTH
    )(
    input clk,
    input reset,
    
    output reg [TIMESTAMP_WIDTH-1:0] timestamp
    );
    
    reg [TIMESTAMP_UNIT_WIDTH-1:0] counter;

    always @(posedge clk) begin
        if (reset) begin
            timestamp <= 0;
            counter <= 0;
        end else begin
            if (counter == TIMESTAMP_UNIT - 1) begin
                timestamp <= timestamp + 1;
                counter <= 0;
            end else begin
                counter <= counter + 1;
            end
        end        
    end

endmodule
