//////////////////////////////////////////////////////////////////////////////////
// Compute the distance for each class in SVM detection using the RBF kernel.
// Accumulates the values for each class for each support vector received.
// If new_computation is set to 1 (with data_valid at 1), the received value will be considered the first (empty old distance)
// Output the current result for each class after 15 periods.
// Pipelined each one period.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

`include "config/svm_parameters.v"

// Do not load the module if not required to avoid compilation errors
`ifdef SVM_KERNEL_TYPE_RBF

`include "config/exp_memory.v"

`define MAX(p,q) ((p)>(q)?(p):(q))
`define PACK_ARRAY(PK_WIDTH,PK_LEN,PK_SRC,PK_DEST) \
if (pack_var < (PK_LEN))\
	assign PK_DEST[(PK_WIDTH)*pack_var+((PK_WIDTH)-1):(PK_WIDTH)*pack_var] = PK_SRC[pack_var];
`define UNPACK_ARRAY(PK_WIDTH,PK_LEN,PK_DEST,PK_SRC) \
if (pack_var < (PK_LEN))\
	assign PK_DEST[pack_var] = PK_SRC[(PK_WIDTH)*pack_var+((PK_WIDTH)-1):(PK_WIDTH)*pack_var];

module svm_kernel_rbf
(
	input [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x,
	input [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] sv,
	input [`SVM_CLASS_WIDTH - 1:0] sv_class,
	input [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln,
	input [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign,
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
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_diff;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_diff;
	reg new_computation_reg_diff;
	
	reg enable_square;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_square;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_square;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_square;
	reg new_computation_reg_square;
	
	reg enable_square_keep [0:1];
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_square_keep [0:1];
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_square_keep [0:1];
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_square_keep [0:1];
	reg new_computation_reg_square_keep [0:1];
	
	reg enable_sum;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_sum;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_sum;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_sum;
	reg new_computation_reg_sum;
	
	reg enable_gamma;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_gamma;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_gamma;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_gamma;
	reg new_computation_reg_gamma;
	
	reg enable_round;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_round;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_round;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_round;
	reg new_computation_reg_round;
	
	reg enable_exp;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_exp;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_ln_reg_exp;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_exp;
	reg new_computation_reg_exp;
		
	reg enable_exp_keep [0:5];
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_exp_keep [0:5];
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_exp_keep [0:5];
	reg new_computation_reg_exp_keep [0:5];

	reg enable_dist;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class_reg_dist;
	reg [2 * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_reg_dist;
	reg new_computation_reg_dist;

	// Unpacked inputs and outputs (arrays in inputs not supported)
	wire [`SVM_PARAM_WIDTH - 1:0] x_vector [0:`SVM_PARAM_COUNT - 1];
	wire [`SVM_PARAM_WIDTH - 1:0] sv_vector [0:`SVM_PARAM_COUNT - 1];
	wire signed [`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC - 1:0] coefs_ln [0:`SVM_CLASS_COUNT - 2];
	wire signed [1:0] coefs_sign [0:`SVM_CLASS_COUNT - 2];
	reg signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] distances [0:`SVM_DECISION_COUNT - 1];

	// Computation variables
	reg signed [`SVM_PARAM_WIDTH + 1 - 1:0] diff_vector [0:`SVM_PARAM_COUNT - 1];
	wire [`SVM_PARAM_WIDTH * 2 - 1:0] square_vector [0:`SVM_PARAM_COUNT - 1];
	wire [`SVM_PARAM_WIDTH * 2 * `SVM_PARAM_COUNT - 1:0] square_vector_packed;
	wire [`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH - 1:0] square_sum_out;
	reg [`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH - 1:0] square_sum;
	reg signed [`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH + `SVM_GAMMA_WIDTH_INT + `SVM_GAMMA_WIDTH_FRAC - 1:0] gamma_sum_full;
	wire signed [`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH + `SVM_GAMMA_WIDTH_INT + `SVM_GAMMA_WIDTH_FRAC - 1:0] gamma_sum_full_neg;
	wire signed [`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH + `SVM_GAMMA_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] gamma_sum_reduced;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] gamma_sum;
	wire signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] exp_input [0:`SVM_CLASS_COUNT - 2];
	wire [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] exp_output [0:`SVM_CLASS_COUNT - 2];
	wire signed [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC + 1 - 1:0] exp_output_signed [0:`SVM_CLASS_COUNT - 2];
	wire signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] exp_output_dist [0:`SVM_CLASS_COUNT - 2];
	wire waiting;
	
	// Generate the input/output (of this and included modules) packing logic
	generate
		genvar pack_var;
		for (pack_var = 0; pack_var < `MAX(`SVM_PARAM_COUNT,`SVM_DECISION_COUNT); pack_var = pack_var + 1) begin: pack_block
			`UNPACK_ARRAY(`SVM_PARAM_WIDTH, `SVM_PARAM_COUNT, x_vector, x_reg_diff)
			`UNPACK_ARRAY(`SVM_PARAM_WIDTH, `SVM_PARAM_COUNT, sv_vector, sv_reg_diff)
			`UNPACK_ARRAY(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC, `SVM_CLASS_COUNT - 1, coefs_ln, coef_ln_reg_exp)
			`UNPACK_ARRAY(2, `SVM_CLASS_COUNT - 1, coefs_sign, coef_sign_reg_dist)
			`PACK_ARRAY(`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC, `SVM_DECISION_COUNT, distances, distance)
			`PACK_ARRAY(`SVM_PARAM_WIDTH * 2, `SVM_PARAM_COUNT, square_vector, square_vector_packed)
		end
	endgenerate	
	
	// Control of the input of each stage of the pipeline
	always @(posedge clk) begin
		// Prepare for diff
		x_reg_diff <= x;
		sv_reg_diff <= sv;
		sv_class_reg_diff <= sv_class;
		coef_ln_reg_diff <= coef_ln;
		coef_sign_reg_diff <= coef_sign;
		new_computation_reg_diff <= new_computation;
		enable_diff <= reset ? 0 : data_valid;
		// Prepare for square
		sv_class_reg_square <= sv_class_reg_diff;
		coef_ln_reg_square <= coef_ln_reg_diff;
		coef_sign_reg_square <= coef_sign_reg_diff;
		new_computation_reg_square <= new_computation_reg_diff;
		enable_square <= reset ? 0 : enable_diff;
		// Delay for square
		sv_class_reg_square_keep[0] <= sv_class_reg_square;
		coef_ln_reg_square_keep[0] <= coef_ln_reg_square;
		coef_sign_reg_square_keep[0] <= coef_sign_reg_square;
		new_computation_reg_square_keep[0] <= new_computation_reg_square;
		enable_square_keep[0] <= reset ? 0 : enable_square;
		sv_class_reg_square_keep[1] <= sv_class_reg_square_keep[0];
		coef_ln_reg_square_keep[1] <= coef_ln_reg_square_keep[0];
		coef_sign_reg_square_keep[1] <= coef_sign_reg_square_keep[0];
		new_computation_reg_square_keep[1] <= new_computation_reg_square_keep[0];
		enable_square_keep[1] <= reset ? 0 : enable_square_keep[0];
		// Prepare for sum		
		sv_class_reg_sum <= sv_class_reg_square_keep[1];
		coef_ln_reg_sum <= coef_ln_reg_square_keep[1];
		coef_sign_reg_sum <= coef_sign_reg_square_keep[1];
		new_computation_reg_sum <= new_computation_reg_square_keep[1];
		enable_sum <= reset ? 0 : enable_square_keep[1];
		// Prepare for gamma
		sv_class_reg_gamma <= sv_class_reg_sum;
		coef_ln_reg_gamma <= coef_ln_reg_sum;
		coef_sign_reg_gamma <= coef_sign_reg_sum;
		new_computation_reg_gamma <= new_computation_reg_sum;
		enable_gamma <= reset ? 0 : enable_sum;
		// Prepare for round
		sv_class_reg_round <= sv_class_reg_gamma;
		coef_ln_reg_round <= coef_ln_reg_gamma;
		coef_sign_reg_round <= coef_sign_reg_gamma;
		new_computation_reg_round <= new_computation_reg_gamma;
		enable_round <= reset ? 0 : enable_gamma;
		// Prepare for exp
		sv_class_reg_exp <= sv_class_reg_round;
		coef_ln_reg_exp <= coef_ln_reg_round;
		coef_sign_reg_exp <= coef_sign_reg_round;
		new_computation_reg_exp <= new_computation_reg_round;
		enable_exp <= reset ? 0 : enable_round;
		// Delay for exp
		sv_class_reg_exp_keep[0] <= sv_class_reg_exp;
		coef_sign_reg_exp_keep[0] <= coef_sign_reg_exp;
		new_computation_reg_exp_keep[0] <= new_computation_reg_exp;
		enable_exp_keep[0] <= reset ? 0 : enable_exp;
		sv_class_reg_exp_keep[1] <= sv_class_reg_exp_keep[0];
		coef_sign_reg_exp_keep[1] <= coef_sign_reg_exp_keep[0];
		new_computation_reg_exp_keep[1] <= new_computation_reg_exp_keep[0];
		enable_exp_keep[1] <= reset ? 0 : enable_exp_keep[0];
		sv_class_reg_exp_keep[2] <= sv_class_reg_exp_keep[1];
		coef_sign_reg_exp_keep[2] <= coef_sign_reg_exp_keep[1];
		new_computation_reg_exp_keep[2] <= new_computation_reg_exp_keep[1];
		enable_exp_keep[2] <= reset ? 0 : enable_exp_keep[1];
		sv_class_reg_exp_keep[3] <= sv_class_reg_exp_keep[2];
		coef_sign_reg_exp_keep[3] <= coef_sign_reg_exp_keep[2];
		new_computation_reg_exp_keep[3] <= new_computation_reg_exp_keep[2];
		enable_exp_keep[3] <= reset ? 0 : enable_exp_keep[2];
		sv_class_reg_exp_keep[4] <= sv_class_reg_exp_keep[3];
		coef_sign_reg_exp_keep[4] <= coef_sign_reg_exp_keep[3];
		new_computation_reg_exp_keep[4] <= new_computation_reg_exp_keep[3];
		enable_exp_keep[4] <= reset ? 0 : enable_exp_keep[3];
		sv_class_reg_exp_keep[5] <= sv_class_reg_exp_keep[4];
		coef_sign_reg_exp_keep[5] <= coef_sign_reg_exp_keep[4];
		new_computation_reg_exp_keep[5] <= new_computation_reg_exp_keep[4];
		enable_exp_keep[5] <= reset ? 0 : enable_exp_keep[4];
		// Prepare for dist
		sv_class_reg_dist <= sv_class_reg_exp_keep[5];
		coef_sign_reg_dist <= coef_sign_reg_exp_keep[5];
		new_computation_reg_dist <= new_computation_reg_exp_keep[5];
		enable_dist <= reset ? 0 : enable_exp_keep[5];
	end
	
	// Following are the computations of the pipeline, ordered
	
	// Compute the square vector (diff and square)
	generate
		genvar i;
		for (i = 0; i < `SVM_PARAM_COUNT; i = i + 1) begin: square_block
			always @(posedge clk) begin
				if (enable_diff)
					diff_vector[i] <= x_vector[i] - sv_vector[i];
				else
					diff_vector[i] <= diff_vector[i];
			end
			square #(
				.VALUE_WIDTH(`SVM_PARAM_WIDTH + 1),
				.VALUE_MAX(1500)
			) param_square ( 
				.value(diff_vector[i]), 
				.square(square_vector[i]), 
				.clk(clk), 
				.reset(reset), 
				.data_valid(enable_square)
			);
		end
	endgenerate
	
	// Sum all squares (sum)
	multi_sum #(
		.VALUE_WIDTH(`SVM_PARAM_WIDTH * 2),
		.VALUE_COUNT(`SVM_PARAM_COUNT),
		.SUM_WIDTH(`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH)
	) square_adder (
		.values(square_vector_packed), 
		.sum(square_sum_out)
	);
	always @(posedge clk) begin
		if (enable_sum)
			square_sum <= square_sum_out;
		else
			square_sum <= square_sum;
	end
	
	// Compute gamma * sum (gamma)
	always @(posedge clk) begin
		if (enable_gamma)
			gamma_sum_full <= `SVM_GAMMA_VALUE * square_sum;
		else
			gamma_sum_full <= gamma_sum_full;
	end
	
	// Compute - gamma * sum and manage overflows (round)
	assign gamma_sum_full_neg = - gamma_sum_full;
	assign gamma_sum_reduced = gamma_sum_full_neg[`SVM_PARAM_WIDTH * 2 + `SVM_PARAM_INDEX_WIDTH + `SVM_GAMMA_WIDTH_INT + `SVM_GAMMA_WIDTH_FRAC - 1:`SVM_GAMMA_WIDTH_FRAC - `EXP_INPUT_WIDTH_FRAC];
	always @(posedge clk) begin
		if (enable_round) begin
			if (gamma_sum_reduced < -2**(`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1))
				gamma_sum <= -2**(`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1);
			else
				gamma_sum <= gamma_sum_reduced[`EXP_INPUT_WIDTH_FRAC + `EXP_INPUT_WIDTH_INT - 1:0];
		end else begin
			gamma_sum <= gamma_sum;
		end
	end
	
	// Compute distance = coef_sign * exp(coef_ln - gamma * sum) for each coef_ln (exp)
	generate
		genvar coef_index;
		for (coef_index = 0; coef_index < `SVM_CLASS_COUNT - 1; coef_index = coef_index + 1) begin: exp_block
			assign exp_input[coef_index] = (coefs_ln[coef_index] + gamma_sum) < -2**(`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1) ? -2**(`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1) : (coefs_ln[coef_index] + gamma_sum);
			exponential exp (
				.x(exp_input[coef_index]),
				.data_valid(enable_exp),
				.y(exp_output[coef_index]), 
				.clk(clk), 
				.reset(reset)
			);
			assign exp_output_signed[coef_index] = (coefs_sign[coef_index] == 0) ? 0 : ((coefs_sign[coef_index] == -1) ? (- exp_output[coef_index]) : exp_output[coef_index]);
			assign exp_output_dist[coef_index][`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC-1:`SVM_DISTANCE_WIDTH_FRAC-`EXP_OUTPUT_WIDTH_FRAC] = exp_output_signed[coef_index];
			assign exp_output_dist[coef_index][`SVM_DISTANCE_WIDTH_FRAC-`EXP_OUTPUT_WIDTH_FRAC-1:0] = 0;
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
					distances[dec_id] <= (new_computation_reg_dist ? 0 : distances[dec_id]) + exp_output_dist[`SVM_DEC_OUT_INDEX1_dec];
				else if (enable_dist && sv_class_reg_dist == `SVM_DEC_SV_CLASS2_dec)
					distances[dec_id] <= (new_computation_reg_dist ? 0 : distances[dec_id]) + exp_output_dist[`SVM_DEC_OUT_INDEX2_dec];
				else if (enable_dist && new_computation_reg_dist)
					distances[dec_id] <= 0;
				else
					distances[dec_id] <= distances[dec_id];
			end
		end
	endgenerate
endmodule

`endif
