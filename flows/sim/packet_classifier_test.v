`timescale 1ns / 1ps

// Packet classifier test

`include "../../config/proj_env.v"
`include "../config/traffic_parameters.v"

`define TEST_PERIOD 1000

module packet_classifier_test;

	// Inputs
	wire [`FLOW_ID_WIDTH-1:0] in_flow_id;
	wire [`PKT_LENGTH_WIDTH-1:0] in_length;
	reg in_wr;
	reg clk;
	reg reset;

	// Outputs
	wire in_ready;
	wire [`FLOW_CLASS_WIDTH-1:0] out_flow_class;
	wire out_wr;
	reg [15:0] pkt_counter;
	reg pkt_counter_enable, pkt_counter_reset;

	// Logic
	reg [2:0] state, state_nxt;
	localparam WAIT = 0;
	localparam SEND = 1;
	localparam INCR = 2;

	// Instantiate the Unit Under Test (UUT)
	packet_classifier uut (
		.in_flow_id(in_flow_id),
		.in_length(in_length),
		.in_wr(in_wr),
		.in_ready(in_ready),
		.out_flow_class(out_flow_class),
		.out_wr(out_wr),
		.clk(clk),
		.reset(reset)
	);

	// Clock
	always begin
	  #5 clk = ~clk;
	end

	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#100;

		// Stop reset
		reset = 0;
	end

	assign in_flow_id = pkt_counter;
	assign in_length = pkt_counter;

	// FSM
	always @(*) begin
		in_wr = 0;
		pkt_counter_reset = 0;
		pkt_counter_enable = 0;
		state_nxt = state;
      case(state)
			WAIT: begin
				if (in_ready) begin
					state_nxt = SEND;
				end
			end
			SEND: begin
				in_wr = 1;
				state_nxt = INCR;
			end
			INCR: begin
				if (pkt_counter >= `TEST_PERIOD - 1)
					pkt_counter_reset = 1;
				else
					pkt_counter_enable = 1;
				state_nxt = WAIT;
			end
      endcase
	end

	// Manage the FSM
	always @(posedge clk) begin
		if (reset)
			state <= WAIT;
		else
			state <= state_nxt;
	end

	// Manage the packet counter
	always @(posedge clk) begin
		if (reset || pkt_counter_reset)
			pkt_counter <= 0;
		else if (pkt_counter_enable)
			pkt_counter <= pkt_counter + 1;
	end
endmodule

