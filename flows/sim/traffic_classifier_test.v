`timescale 1ns / 1ps

module traffic_classifier_test;

	// Inputs
	reg out_rdy;
	reg [63:0] in_data;
	reg [7:0] in_ctrl;
	reg in_wr;
	reg reg_req_in;
	reg reg_ack_in;
	reg reg_rd_wr_L_in;
	reg [22:0] reg_addr_in;
	reg [31:0] reg_data_in;
	reg [1:0] reg_src_in;
	reg clk;
	reg reset;

	// Outputs
	wire [63:0] out_data;
	wire [7:0] out_ctrl;
	wire out_wr;
	wire in_rdy;
	wire reg_req_out;
	wire reg_ack_out;
	wire reg_rd_wr_L_out;
	wire [22:0] reg_addr_out;
	wire [31:0] reg_data_out;
	wire [1:0] reg_src_out;
	
	// Logic
	reg [31:0] counter;
	reg [31:0] global_counter;
	reg [1:0] state, state_next;
	reg counter_reset;
    reg counter_en;
	
	localparam STATE_RESET = 0;
	localparam STATE_SEND = 1;
	localparam STATE_WAIT = 2;
    
    // SRAM connection
    wire rd_0_req, wr_0_req, rd_0_ack, rd_0_vld, wr_0_ack;
	wire [71:0] rd_0_data, wr_0_data;
	wire [18:0] rd_0_addr, wr_0_addr;    

	// Instantiate the Unit Under Test (UUT)
	traffic_classifier uut (
		.out_data(out_data), 
		.out_ctrl(out_ctrl), 
		.out_wr(out_wr), 
		.out_rdy(out_rdy), 
		.in_data(in_data), 
		.in_ctrl(in_ctrl), 
		.in_wr(in_wr), 
		.in_rdy(in_rdy), 
		.reg_req_in(reg_req_in), 
		.reg_ack_in(reg_ack_in), 
		.reg_rd_wr_L_in(reg_rd_wr_L_in), 
		.reg_addr_in(reg_addr_in), 
		.reg_data_in(reg_data_in), 
		.reg_src_in(reg_src_in), 
		.reg_req_out(reg_req_out), 
		.reg_ack_out(reg_ack_out), 
		.reg_rd_wr_L_out(reg_rd_wr_L_out), 
		.reg_addr_out(reg_addr_out), 
		.reg_data_out(reg_data_out), 
		.reg_src_out(reg_src_out), 
		.clk(clk), 
		.reset(reset),
        .wr_0_addr(wr_0_addr),
        .wr_0_req(wr_0_req),
        .wr_0_data(wr_0_data),
        .wr_0_ack(wr_0_ack),
        .rd_0_ack(rd_0_ack),
        .rd_0_data(rd_0_data),
        .rd_0_vld(rd_0_vld),
        .rd_0_addr(rd_0_addr),
        .rd_0_req(rd_0_req)
	);
    
    // Fake SRAM
	fake_netfpga_sram sram (
		.clk(clk), 
		.reset(reset),
        .wr_0_addr(wr_0_addr),
        .wr_0_req(wr_0_req),
        .wr_0_data(wr_0_data),
        .wr_0_ack(wr_0_ack),
        .rd_0_ack(rd_0_ack),
        .rd_0_data(rd_0_data),
        .rd_0_vld(rd_0_vld),
        .rd_0_addr(rd_0_addr),
        .rd_0_req(rd_0_req)
	);

	// Initialization
	initial begin
		// Initialize Inputs
		in_data = 0;
		in_ctrl = 0;
		in_wr = 0;
		reg_req_in = 0;
		reg_ack_in = 0;
		reg_rd_wr_L_in = 0;
		reg_addr_in = 0;
		reg_data_in = 0;
		reg_src_in = 0;
		clk = 1;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Start
		reset = 0;
	end
   
	// Clock
	always begin
	  #5 clk = ~clk;
	end
	
	// FSM 
	always @(*) begin
		state_next = state;
		counter_reset = 0;
        counter_en = 0;
		in_data = #1 0;
		in_ctrl = #1 0;
		in_wr = #1 0;
		out_rdy = #1 (global_counter % 5) != 0;
		case (state)
			STATE_RESET: begin
				if (!reset)
					state_next = STATE_WAIT;
			end
			STATE_SEND: begin
                in_wr = #1 1;
                counter_en = 1;
                case (counter)
                    0: begin
                        in_ctrl = #1 8'hFF;
                        in_data = #1 64'h0;
                    end
                    1: begin
                        in_data = #1 64'h0023ebf960000017;
                    end
                    2: begin
                        in_data = #1 64'ha44081e408004510;
                    end
                    3: begin
                        in_data = #1 64'h0058070e40004006;
                    end
                    4: begin
                        in_data = #1 64'hb98dc06c759493e5;
                    end
                    5: begin
                        in_data = #1 64'hb00ece870016cec4;
                    end
                    6: begin
                        in_data = #1 64'hc8fd4b7b71175018;
                    end
                    7: begin
                        in_data = #1 64'hf53c23e00000db58;
                    end
                    8: begin
                        in_data = #1 64'h0a289714de6d048b;
                    end
                    9: begin
                        in_data = #1 64'h48bcc280fcd9e494;
                    end
                    10: begin
                        in_data = #1 64'he518a1b9e62c7980;
                    end
                    11,12,13: begin
                        in_data = #1 0;
                    end
                    14: begin
                        in_data = #1 0;
                        in_ctrl = #1 8'hFF;
                    end
                    15: begin
                        in_data = #1 64'h0017a44081e40023;
                    end
                    16: begin
                        in_data = #1 64'hebf9600008004500;
                    end
                    17: begin
                        in_data = #1 64'h004c000040002c11;
                    end
                    18: begin
                        in_data = #1 64'h5edf5bbd5e04c06c;
                    end
                    19: begin
                        in_data = #1 64'h7594007b007b0038;
                    end
                    20: begin
                        in_data = #1 64'h190d240206ec0000;
                    end
                    21: begin
                        in_data = #1 64'h021f00000be6c14f;
                    end
                    22, 23, 24, 25: begin
                        in_data = #1 0;
                    end
                    26, 27, 28, 29, 30, 31: begin
                        in_data = #1 0;
                        in_wr = #1 0;
                    end
                    32: begin
                        in_data = #1 0;
                        in_wr = #1 0;
                        counter_reset = 1;
                    end
                endcase
				if (!in_rdy)
                    state_next = STATE_WAIT;
			end
            STATE_WAIT: begin
                if (in_rdy)
                    state_next = STATE_SEND;
            end
		endcase
	end
	
	// Counter management
	always @(posedge clk) begin
		if (reset || counter_reset)
			counter <= 0;
		else if (counter_en)
			counter <= counter + 1;
	end
	
	// Global counter management
	always @(posedge clk) begin
		if (reset)
			global_counter <= 0;
		else
			global_counter <= global_counter + 1;
	end
	
	// State management
	always @(posedge clk) begin
		if (reset)
			state <= STATE_RESET;
		else
			state <= state_next;
	end
	
	
endmodule

