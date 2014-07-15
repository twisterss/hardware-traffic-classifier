`timescale 1ns / 1ps

// Flow lookup test

`include "../../config/proj_env.v"
`include "../config/traffic_parameters.v"

module flow_lookup_test;

	// Inputs
	reg [`FLOW_ID_WIDTH-1:0] in_flow_id;
	reg in_wr;
	reg reset;
	reg clk;

	// Outputs
	wire [`FLOW_ADDR_WIDTH-1:0] out_flow_addr;
	wire out_flow_new;
	wire out_wr;
	wire in_ready;
	
	// Logic
	reg [`FLOW_ID_WIDTH-1:0] counter;
	reg [1:0] state, state_next;
	
	localparam STATE_WAIT = 0;
	localparam STATE_SEND = 1;

	// Instantiate the Unit Under Test (UUT)
	flow_lookup uut (
		.in_flow_id(in_flow_id), 
		.in_wr(in_wr), 
		.in_ready(in_ready),
		.out_flow_addr(out_flow_addr), 
		.out_flow_new(out_flow_new), 
		.out_wr(out_wr), 
		.reset(reset), 
		.clk(clk)
	);

	initial begin
		// Initialize Inputs
		reset = 1;
		clk = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Stop reset
		reset = 0;
	end
   
	// Clock
	always begin
	  #5 clk = ~clk;
	end
	
	// FSM 
	always @(*) begin
		state_next = state;
		in_wr = 0;
		in_flow_id = 0;
		case (state)
			STATE_WAIT: begin
				if (in_ready)
					state_next = STATE_SEND;
			end
			STATE_SEND: begin
				in_wr = 1;
				in_flow_id = counter;
				state_next = STATE_WAIT;
			end
		endcase
	end
	
	// Counter management
	always @(posedge clk) begin
		if (reset)
			counter <= 0;
		else
			counter <= counter + 1;
	end
	
	// State management
	always @(posedge clk) begin
		if (reset)
			state <= STATE_WAIT;
		else
			state <= state_next;
	end
      
endmodule

