`timescale 1ns / 1ps

// Test bench of the flow storage module

`include "../../config/proj_env.v"
`include "../config/traffic_parameters.v"

module tb_flow_storage;

	// Inputs
	reg clk;
	reg reset;
	reg write_en;   
    reg [`FLOW_RAM_EDIT_ID_WIDTH-1:0] write_edit_id;
    reg [`FLOW_DATA_WIDTH-1:0] write_data;
    reg read_en;
    reg [`FLOW_ID_WIDTH-1:0] read_id;

	// Outputs
	wire write_ready;
    wire read_ready;
    wire [`FLOW_DATA_WIDTH-1:0] read_data;
    wire [`FLOW_RAM_EDIT_ID_WIDTH-1:0] read_edit_id;
    wire read_data_new;
    wire read_data_found;
    
    // Internal
    reg read_old;
    reg [`FLOW_ID_WIDTH-1:0] write_id;
    reg write_old;
    reg error;
    
    // SRAM connection (if any)
    `ifdef NETFPGA_1G
        wire rd_0_req, wr_0_req, rd_0_ack, rd_0_vld, wr_0_ack;
        wire [`FLOW_RAM_WORD_WIDTH-1:0] rd_0_data, wr_0_data;
        wire [`FLOW_RAM_ADDR_WIDTH-1:0] rd_0_addr, wr_0_addr; 
    `endif

	// Instantiate the Unit Under Test (UUT)
	flow_storage uut (
		.clk(clk), 
		.reset(reset), 
		.write_ready(write_ready), 
		.write_en(write_en),
        .write_edit_id(write_edit_id),
        .write_data(write_data),
        .read_ready(read_ready),
        .read_en(read_en),
        .read_id(read_id),
        .read_edit_id(read_edit_id),
        .read_data(read_data),
        .read_data_new(read_data_new),
        .read_data_found(read_data_found)
        `ifdef NETFPGA_1G
            ,
            .wr_0_addr(wr_0_addr),
            .wr_0_req(wr_0_req),
            .wr_0_data(wr_0_data),
            .wr_0_ack(wr_0_ack),
            .rd_0_ack(rd_0_ack),
            .rd_0_data(rd_0_data),
            .rd_0_vld(rd_0_vld),
            .rd_0_addr(rd_0_addr),
            .rd_0_req(rd_0_req)
        `endif
	);
    
    // Fake SRAM (if any)
    `ifdef NETFPGA_1G
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
    `endif
    
    // Clock simulation
    always begin
        #5 clk = !clk;
    end

	initial begin
		// Initialize Inputs
		clk = 1;
		reset = 1;
		write_en = 0;
        write_data = 0;
        read_en = 0;
        read_id = 0;
        read_old = 0;
        write_id = 1;
        write_old = 0;
        error = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Stop reset
        reset = 0;
	end
    
    // Read memory incrementing ids
    always @(posedge clk) begin
        read_en <= read_ready;
        if (read_ready) begin
            if (!read_old && read_id <= 10) begin
                read_id <= read_id + 1;
            end else begin
                if (!read_old)
                    read_id <= read_id - 10;
                else
                    read_id <= read_id + 11;
                read_old <= !read_old;
            end
        end
    end

    // Save edit_id during read to use during write
    always @(posedge clk) begin
        // Write if not old
        write_en <= read_data_new && !write_old;
        if (read_data_new && !write_old) begin
            write_edit_id <= read_edit_id;
            write_data <= write_id;
        end
        // Keep the same counter as in read
        if (read_data_new) begin
            if (!write_old && write_id <= 10) begin
                write_id <= write_id + 1;
            end else begin
                if (!write_old)
                    write_id <= write_id - 10;
                else
                    write_id <= write_id + 11;
                write_old <= !write_old;
            end
        end
        // Detect errors
        if (read_data_new && write_old)
            error <= error || read_data != write_id;
    end
endmodule

