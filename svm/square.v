// Compute a square without multiplier.
// Require a ROM.
// The result is given two periods late.
// Pipeline: each period.

`timescale 1ns / 1ps

module square
#(
	parameter VALUE_WIDTH = 12,
	parameter VALUE_MAX = 1500
)
(
	input signed [VALUE_WIDTH - 1:0] value,
	output reg [(VALUE_WIDTH - 1) * 2 - 1:0] square,
	output reg new_result,
	
	input clk,
	input reset,
	input data_valid
);
	// Pipeline stages inputs
	reg enable_addr;
	reg signed [VALUE_WIDTH - 1:0] value_reg_addr;
	
	reg enable_read;

	// Internal variables
	reg [VALUE_WIDTH - 2:0] addr;
	reg [VALUE_WIDTH * 2 - 1:0] rom [0:VALUE_MAX];

	// Initialize the ROM values
	reg [VALUE_WIDTH - 1:0] i;
	initial begin
		for (i = 0; i <= VALUE_MAX; i = i + 1) begin
			rom[i] = i * i;
		end
	end
	
	// Pipeline control
	always @(posedge clk) begin
		// Prepare for addr
		enable_addr <= reset ? 0 : data_valid;
		value_reg_addr <= value;
		// Prepare for read
		enable_read <= reset ? 0 : enable_addr;
	end
	
	// Address computation (addr)
	always @(posedge clk) begin
		if (enable_addr)
			addr <= (value_reg_addr < - VALUE_MAX || value_reg_addr > VALUE_MAX) ? VALUE_MAX : ((value_reg_addr < 0) ? (- value_reg_addr) : value_reg_addr);
		else
			addr <= addr;
	end
	
	// Control the ROM (read)
	always @(posedge clk) begin
		if (reset) begin
			square <= 0;
			new_result <= 0;
		end else if (enable_read) begin
			square <= rom[addr];
			new_result <= 1;
		end else begin
			square <= square;
			new_result <= 0;
		end
	end

endmodule
