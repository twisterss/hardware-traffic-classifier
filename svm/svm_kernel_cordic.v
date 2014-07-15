//////////////////////////////////////////////////////////////////////////////////
// Compute the distance for each class in SVM detection using the CORDIC kernel.
// Accumulates the values for each class for each support vector received.
// If new_computation is set to 1 (with data_valid at 1), the received value will be considered the first (empty old distance)
// Output the current result for each class after 24 periods.
// Pipelined each one period.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

`include "config/svm_parameters.v"

// Do not load the module if not required to avoid compilation errors
`ifdef SVM_KERNEL_TYPE_CORDIC

`define MAX(p,q) ((p)>(q)?(p):(q))
`define PACK_ARRAY(PK_WIDTH,PK_LEN,PK_SRC,PK_DEST) \
if (pack_var < (PK_LEN))\
	assign PK_DEST[(PK_WIDTH)*pack_var+((PK_WIDTH)-1):(PK_WIDTH)*pack_var] = PK_SRC[pack_var];
`define UNPACK_ARRAY(PK_WIDTH,PK_LEN,PK_DEST,PK_SRC) \
if (pack_var < (PK_LEN))\
	assign PK_DEST[pack_var] = PK_SRC[(PK_WIDTH)*pack_var+((PK_WIDTH)-1):(PK_WIDTH)*pack_var];

module svm_kernel_cordic
(
	input [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x,
	input [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] sv,
	input [`SVM_CLASS_WIDTH - 1:0] sv_class,
	input [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_scaled,
	input [`SVM_CLASS_COUNT - 2:0] coef_sign,
	input new_computation,

	input clk,
	input reset,
	input data_valid,

	output [(`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC) * `SVM_DECISION_COUNT - 1:0] distance
);

	// Registered inputs for each pipeline step
	reg enable_diff;
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x_reg_diff;
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] sv_reg_diff;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_diff;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_scaled_reg_diff;
	reg [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_diff;
	reg new_computation_reg_diff;

	reg enable_sum;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_sum;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_scaled_reg_sum;
	reg [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_sum;
	reg new_computation_reg_sum;

	reg enable_ceil;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_ceil;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_scaled_reg_ceil;
	reg [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_ceil;
	reg new_computation_reg_ceil;

	reg enable_cord;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_cord;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_scaled_reg_cord;
	reg [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_cord;
	reg new_computation_reg_cord;

	reg enable_shift;
	wire [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_shift;
	wire [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_shift;
	wire new_computation_reg_shift;
	wire [`SVM_CORDIC_WIDTH_LN - 1:0] cordic_shift_reg_shift;

	reg enable_dist;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_dist;
	reg [(`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_dist;
	reg new_computation_reg_dist;

	// Unpacked inputs and outputs (arrays in inputs not supported)
	wire [`SVM_PARAM_WIDTH - 1:0] x_vector [0:`SVM_PARAM_COUNT - 1];
	wire [`SVM_PARAM_WIDTH - 1:0] sv_vector [0:`SVM_PARAM_COUNT - 1];
	wire signed [`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC - 1:0] coefs_scaled [0:`SVM_CLASS_COUNT - 2];
	reg signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] distances [0:`SVM_DECISION_COUNT - 1];

	// Computation variables
	wire signed [`SVM_PARAM_WIDTH + 1 - 1:0] diff_vector [0:`SVM_PARAM_COUNT - 1];
	reg [`SVM_PARAM_WIDTH - 1:0] diff_vector_abs [0:`SVM_PARAM_COUNT - 1];
	wire [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] diff_vector_packed;
	wire [`SVM_PARAM_WIDTH + `SVM_PARAM_INDEX_WIDTH - 1:0] diff_sum_out;
	reg [`SVM_PARAM_WIDTH + `SVM_PARAM_INDEX_WIDTH - 1:0] diff_sum;
	wire [`SVM_PARAM_WIDTH + `SVM_PARAM_INDEX_WIDTH - 1:0] cordic_shift_tmp;
	reg [`SVM_CORDIC_WIDTH_LN - 1:0] cordic_shift;
	reg [`SVM_CORDIC_WIDTH - 1:0] cordic_input;
	wire [`SVM_CORDIC_WIDTH - 1:0] cordic_output [0:`SVM_CLASS_COUNT - 2];
	wire cordic_output_rdy_nxt;
	reg [`SVM_CORDIC_WIDTH - 1:0] cordic_output_shifted [0:`SVM_CLASS_COUNT - 2];

	// Generate the input/output (of this and included modules) packing logic
	generate
		genvar pack_var;
		for (pack_var = 0; pack_var < `MAX(`SVM_PARAM_COUNT,`SVM_DECISION_COUNT); pack_var = pack_var + 1) begin: pack_block
			`UNPACK_ARRAY(`SVM_PARAM_WIDTH, `SVM_PARAM_COUNT, x_vector, x_reg_diff)
			`UNPACK_ARRAY(`SVM_PARAM_WIDTH, `SVM_PARAM_COUNT, sv_vector, sv_reg_diff)
			`UNPACK_ARRAY(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC, `SVM_CLASS_COUNT - 1, coefs_scaled, coef_scaled_reg_cord)
			`PACK_ARRAY(`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC, `SVM_DECISION_COUNT, distances, distance)
			`PACK_ARRAY(`SVM_PARAM_WIDTH, `SVM_PARAM_COUNT, diff_vector_abs, diff_vector_packed)
		end
	endgenerate

	// Control of the input of each stage of the pipeline
	always @(posedge clk) begin
		// Prepare for diff
		x_reg_diff <= x;
		sv_reg_diff <= sv;
		sv_class_reg_diff <= sv_class;
		coef_scaled_reg_diff <= coef_scaled;
		coef_sign_reg_diff <= coef_sign;
		new_computation_reg_diff <= new_computation;
		enable_diff <= reset ? 0 : data_valid;
		// Prepare for sum
		sv_class_reg_sum <= sv_class_reg_diff;
		coef_scaled_reg_sum <= coef_scaled_reg_diff;
		coef_sign_reg_sum <= coef_sign_reg_diff;
		new_computation_reg_sum <= new_computation_reg_diff;
		enable_sum <= enable_diff;
		// Prepare for ceil
		sv_class_reg_ceil <= sv_class_reg_sum;
		coef_scaled_reg_ceil <= coef_scaled_reg_sum;
		coef_sign_reg_ceil <= coef_sign_reg_sum;
		new_computation_reg_ceil <= new_computation_reg_sum;
		enable_ceil <= enable_sum;
		// Prepare for cord
		sv_class_reg_cord <= sv_class_reg_ceil;
		coef_scaled_reg_cord <= coef_scaled_reg_ceil;
		coef_sign_reg_cord <= coef_sign_reg_ceil;
		new_computation_reg_cord <= new_computation_reg_ceil;
		enable_cord <= enable_ceil;
		// Variable delay for cord (see the FIFO below)
		enable_shift <= cordic_output_rdy_nxt;
		// Prepare for dist
		sv_class_reg_dist <= sv_class_reg_shift;
		coef_sign_reg_dist <= coef_sign_reg_shift;
		coef_sign_reg_dist <= coef_sign_reg_shift;
		new_computation_reg_dist <= new_computation_reg_shift;
		enable_dist <= reset ? 0 : enable_shift;
	end

	// Variable-duration wait for the CORDIC using a FIFO
	// The FIFO depth should be at least the CORDIC duration (in clock cycles)
	small_fifo #(
		.WIDTH(`SVM_CLASS_WIDTH + (`SVM_CLASS_COUNT-1) + 1 + `SVM_CORDIC_WIDTH_LN),
		.MAX_DEPTH_BITS(`SVM_CORDIC_STEP_WIDTH)
	) cord_fifo (
		.din({sv_class_reg_cord, coef_sign_reg_cord, new_computation_reg_cord, cordic_shift}),
		.dout({sv_class_reg_shift, coef_sign_reg_shift, new_computation_reg_shift, cordic_shift_reg_shift}),
		.wr_en(enable_cord),
		.rd_en(cordic_output_rdy_nxt),
        .clk(clk),
        .reset(reset)
	);

	// Following are the computations of the pipeline, ordered

	// Compute the absolute difference for each parameter (diff)
	generate
		genvar i;
		for (i = 0; i < `SVM_PARAM_COUNT; i = i + 1) begin: square_block
			assign diff_vector[i] = x_vector[i] - sv_vector[i];
			always @(posedge clk) begin
				diff_vector_abs[i] <= diff_vector[i] >= 0 ? diff_vector[i] : -diff_vector[i];
			end
		end
	endgenerate

	// Sum all differences and shift for gamma (sum)
	// Cordic input and shift are ready at this time
	multi_sum #(
		.VALUE_WIDTH(`SVM_PARAM_WIDTH),
		.VALUE_COUNT(`SVM_PARAM_COUNT),
		.SUM_WIDTH(`SVM_PARAM_WIDTH + `SVM_PARAM_INDEX_WIDTH)
	) diff_adder (
		.values(diff_vector_packed),
		.sum(diff_sum_out)
	);
	always @(posedge clk) begin
		diff_sum <= diff_sum_out;
	end

	// Compute the CORDIC shift and input (ceil)
	// Cordic input and shift are ready at this time
    assign cordic_shift_tmp = diff_sum[`SVM_PARAM_WIDTH + `SVM_PARAM_INDEX_WIDTH - 1 : `SVM_LOG_GAMMA];
	always @(posedge clk) begin
		cordic_shift <= cordic_shift_tmp > `SVM_CORDIC_WIDTH ? `SVM_CORDIC_WIDTH : cordic_shift_tmp;
		cordic_input <= {diff_sum[`SVM_LOG_GAMMA-1:0], {(`SVM_CORDIC_WIDTH-`SVM_LOG_GAMMA){1'b0}}};
	end

	// Compute the CORDIC value for each coef_scaled (cord)
	generate
		genvar coef_index;
		for (coef_index = 0; coef_index < `SVM_CLASS_COUNT - 1; coef_index = coef_index + 1) begin: cordic_block
			wire data_rdy_nxt;
			cordic cordic_computer (
                .enable(enable_cord),
				.x(cordic_input),
				.y(coefs_scaled[coef_index]),
				.result(cordic_output[coef_index]),
				.data_valid_nxt(data_rdy_nxt),
				.clk(clk),
				.reset(reset)
			);
			if (coef_index == 0)
				assign cordic_output_rdy_nxt = data_rdy_nxt;
		end
	endgenerate

	// Shift the CORDIC value for each coef_scaled (shift)
	generate
		for (coef_index = 0; coef_index < `SVM_CLASS_COUNT - 1; coef_index = coef_index + 1) begin: shift_block
			always @(posedge clk) begin
				cordic_output_shifted[coef_index] <= cordic_output[coef_index] >> cordic_shift_reg_shift;
			end
		end
	endgenerate

	// Global distances management (dist)
	generate
		genvar dec_id;
		for (dec_id = 0; dec_id < `SVM_DECISION_COUNT; dec_id = dec_id + 1) begin: reset_block
			always @(posedge clk) begin
				if (reset)
					distances[dec_id] <= 0;
				else if (enable_dist && sv_class_reg_dist == `SVM_DEC_SV_CLASS1_dec)
					distances[dec_id] <= (new_computation_reg_dist ? 0 : distances[dec_id]) + (coef_sign_reg_dist[`SVM_DEC_OUT_INDEX1_dec] ? -cordic_output_shifted[`SVM_DEC_OUT_INDEX1_dec] : cordic_output_shifted[`SVM_DEC_OUT_INDEX1_dec]);
				else if (enable_dist && sv_class_reg_dist == `SVM_DEC_SV_CLASS2_dec)
					distances[dec_id] <= (new_computation_reg_dist ? 0 : distances[dec_id]) + (coef_sign_reg_dist[`SVM_DEC_OUT_INDEX2_dec] ? -cordic_output_shifted[`SVM_DEC_OUT_INDEX2_dec] : cordic_output_shifted[`SVM_DEC_OUT_INDEX2_dec]);
				else if (enable_dist && new_computation_reg_dist)
					distances[dec_id] <= 0;
				else
					distances[dec_id] <= distances[dec_id];
			end
		end
	endgenerate
endmodule

`endif