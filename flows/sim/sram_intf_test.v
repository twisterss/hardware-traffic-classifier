`timescale 1ns / 1ps


`include "../../config/proj_env.v"
`include "../config/traffic_parameters.v"

module sram_intf_test;

	// Inputs
	reg clk;
	reg reset;
	reg write_en;
	reg [`FLOW_RAM_ADDR_WIDTH-1:0] write_addr;
	reg [`FLOW_RAM_WORD_WIDTH-1:0] write_data;
	reg read_en;
	reg [`FLOW_RAM_ADDR_WIDTH-1:0] read_addr;

	// Outputs
	wire write_ready;
	wire read_ready;
	wire [`FLOW_RAM_WORD_WIDTH-1:0] read_data;
	wire read_data_new;
     

	// Instantiate the Unit Under Test (UUT)
	sram_reset_intf uut (
		.clk(clk), 
		.reset(reset), 
		.write_ready(write_ready), 
		.write_en(write_en), 
		.write_addr(write_addr), 
		.write_data(write_data), 
		.read_ready(read_ready), 
		.read_en(read_en), 
		.read_addr(read_addr), 
		.read_data(read_data), 
		.read_data_new(read_data_new)
	);

	initial begin
		// Initialize Inputs
		reset = 1;
        write_data = 0;
        write_addr = 0;
        write_en = 0;
        read_addr = 0;
        read_en = 0;
		clk = 1;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Stop reset
		reset = 0;
        #21
        read_addr = 5;
        read_en = 1;
        write_data = 547856;
        write_addr = 12;
        write_en = 1;
        
        #20
        read_addr = 0;
        read_en = 0;
        write_data = 0;
        write_addr = 0;
        write_en = 0;
        
        #30
        read_addr = 12;
        read_en = 1;
        
        #20
        read_addr = 0;
        read_en = 0;
        
        #30
        write_data = 123456;
        write_addr = 25;
        write_en = 1;
        read_addr = 12;
        read_en = 1;
        
        #10
        write_data = 987654;
        write_addr = {(`FLOW_RAM_ADDR_WIDTH){1'b1}};
        write_en = 1;
        read_addr = {(`FLOW_RAM_ADDR_WIDTH){1'b1}};
        read_en = 1;
        
        #10
        write_data = 27;
        write_addr = 42;
        write_en = 1;
        read_addr = {(`FLOW_RAM_ADDR_WIDTH){1'b1}};
        read_en = 1;    
        
        #10
        write_data = 0;
        write_addr = 0;
        write_en = 0;
        read_addr = 0;
        read_en = 0;       
	end
      
	// Clock
	always begin
	  #5 clk = ~clk;
	end
    
endmodule

