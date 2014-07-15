//////////////////////////////////////////////////////////////////////////////////
// SVM detection module
// Take a vector and return the detected class (from 0 to N).
// Some constants should be less than 100: SVM_SV_UNROLL, SVM_CLASS_COUNT, SVM_PARAM_COUNT
// May take input only when ready was set to 1 at last clock cycle
// Pipeline steps:
// * read: read the memory for SVM model
// * compute: computation done by svm_kernel
// * keep: keep the computed distance until all iterations are done
// * sum: sum all distances from each iteration
// * compare: compare distances to rho
// * vote: compute the number of votes for each class
// * select: select the class with the most votes
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

`include "config/svm_parameters.v"
`include "../config/proj_env.v"

// Duration of the kernel computation pipeline (depends on the kernel)
`ifdef SVM_KERNEL_TYPE_RBF
`define KERNEL_DURATION 15
`endif
`ifdef SVM_KERNEL_TYPE_CORDIC
`define KERNEL_DURATION 24
`endif

// Number of SVs for each loop
`define SVM_SV_COUNT_r ((r == (`SVM_SV_UNROLL - 1)) ? `SVM_SV_COUNT_UNROLLED_LAST : `SVM_SV_COUNT_UNROLLED)

// Conversion from int to ASCII string (works only for 1 or 2 digits numbers)
// Used for unroll, parameter and class number
// This is a trick because sformat causes synthesis errors in XST
`define INT_TO_STR10(v) ((v) == 0 ? "0" : (v) == 1 ? "1" : (v) == 2 ? "2" : (v) == 3 ? "3" : (v) == 4 ? "4" : (v) == 5 ? "5" : (v) == 6 ? "6" : (v) == 7 ? "7" : (v) == 8 ? "8" : "9")
`define INT_TO_STR100(v) {`INT_TO_STR10(v/10),`INT_TO_STR10(v%10)}

module svm_detection
(
    input [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x,
	 input data_valid,

    input clk,
    input reset,

    output [`SVM_CLASS_WIDTH - 1:0] class,
	 output new_result,
    output ready
);

	// The distance computation is parallelized (read, compute, keep)
	// Global output of this part of the pipeline
	reg distance_ready_keep;
	reg signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] distances_keep [0:`SVM_SV_UNROLL-1][0:`SVM_DECISION_COUNT-1];
	// Iterations of the distances computation
	generate
		genvar r, p, c, k, d;
		for (r = 0; r < `SVM_SV_UNROLL; r = r + 1) begin: roll
			// Memories declaration and initialization
			// Parameters of each SV
			for (p = 0; p < `SVM_PARAM_COUNT; p = p + 1) begin: param
				reg [`SVM_PARAM_WIDTH - 1 : 0] sv_param [0 : `SVM_SV_COUNT_r - 1];
				initial begin: init_param
					// 48 is the ASCII number for 0
					// Used for all readmemh because sformat causes synthesis errors
					// This works only for 1 or 2 chars, so 0 to 99
					$readmemh({`PROJ_CONFIG_DIR, "svm_parameters.sv.", `INT_TO_STR100(r), ".param.", `INT_TO_STR100(p), ".list"}, sv_param);
				end
			end
			// Class of each SV
			reg [`SVM_CLASS_WIDTH - 1 : 0] sv_class [0 : `SVM_SV_COUNT_r - 1];
			initial begin: init_class
				$readmemh({`PROJ_CONFIG_DIR, "svm_parameters.sv.", `INT_TO_STR100(r), ".class.list"}, sv_class);
			end
			// Coefficients of each SV
			for (c = 0; c < `SVM_CLASS_COUNT - 1; c = c + 1) begin: coef
				reg [`SVM_COEF_WIDTH_FRAC + `SVM_COEF_WIDTH_INT - 1 : 0] sv_coef [0 : `SVM_SV_COUNT_r - 1];
				// The sign is stored on 2 bits: always enough (1 => internal error in synthesis)
				reg [1:0] sv_coef_sign [0 : `SVM_SV_COUNT_r - 1];
				initial begin: init_coef
					$readmemh({`PROJ_CONFIG_DIR, "svm_parameters.sv.", `INT_TO_STR100(r), ".coef.", `INT_TO_STR100(c), ".list"}, sv_coef);
					$readmemh({`PROJ_CONFIG_DIR, "svm_parameters.sv.", `INT_TO_STR100(r), ".coef_sign.", `INT_TO_STR100(c), ".list"}, sv_coef_sign);
				end
			end
			// Kernel computation FSM
			reg [`SVM_SV_COUNT_UNROLLED_WIDTH - 1:0] counter_read;
			reg enable_read;
			reg enable_compute;
			reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x_read;
			always @(posedge clk) begin
				if (reset) begin
					counter_read <= 0;
					enable_read <= 0;
					enable_compute <= 0;
					x_read <= 0;
				end else	begin
					if (ready && data_valid) begin
						counter_read <= 1; //0 is already pre-read!
						enable_read <= 1; // is 0 at least once each computation
						enable_compute <= 1;
						x_read <= x;
					end else begin
						enable_compute <= enable_read;
						if (enable_read) begin
							if ((counter_read + 1) < `SVM_SV_COUNT_r) begin
								counter_read <= counter_read + 1;
								enable_read <= 1;
							end else begin
								counter_read <= 0;
								enable_read <= 0;
							end
						end else begin
							counter_read <= 0;
							enable_read <= 0;
						end
					end
				end
			end
			// Ready signal management by the first unrolled loop (always longest)
			// The ready signal is set from the period when the last data is read from memory
			// It is anticipated because a synchronized reaction is assumed!
			if (r == 0) begin
				assign ready = ((!enable_read) || (counter_read == `SVM_SV_COUNT_r - 1));
			end
			// Memory read at each step (even if no data to stay ready)
			reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] sv_read;
			for (p = 0; p < `SVM_PARAM_COUNT; p = p + 1) begin: param_read
				always @(posedge clk) begin
					sv_read[`SVM_PARAM_WIDTH*(p+1)-1:`SVM_PARAM_WIDTH*p] <= reset ? 0 : param[p].sv_param[counter_read];
				end
			end
			reg [`SVM_CLASS_WIDTH - 1:0] sv_class_read;
			always @(posedge clk) begin
				sv_class_read <= reset ? 0 : sv_class[counter_read];
			end
			reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef_read;
			reg [`SVM_COEF_SIGN_WIDTH * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign_read;
			for (c = 0; c < `SVM_CLASS_COUNT - 1; c = c + 1) begin: coef_read_block
				always @(posedge clk) begin
					coef_read[(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (c+1) - 1:(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * c] <= reset ? 0 : coef[c].sv_coef[counter_read];
					coef_sign_read[`SVM_COEF_SIGN_WIDTH * (c+1) - 1:`SVM_COEF_SIGN_WIDTH * c] <= reset ? 0 : coef[c].sv_coef_sign[counter_read];
				end
			end
			// Connect the svm_kernel block
			wire [(`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC) * (`SVM_DECISION_COUNT) - 1:0] distance_computed;
			wire last_compute;
			wire new_computation_read;
			assign last_compute = enable_compute && !enable_read;
			assign new_computation_read = enable_read && counter_read == 1;
			`ifdef SVM_KERNEL_TYPE_RBF
                // RBF kernel computation
                svm_kernel_rbf kernel_computation (
                    .x(x_read),
                    .sv(sv_read),
                    .sv_class(sv_class_read),
                    .coef_ln(coef_read),
                    .coef_sign(coef_sign_read),
                    .new_computation(new_computation_read),
                    .data_valid(enable_compute),
                    .clk(clk),
                    .reset(reset),
                    .distance(distance_computed)
                );
            `endif
            `ifdef SVM_KERNEL_TYPE_CORDIC
                // CORDIC kernel computation
                svm_kernel_cordic kernel_computation (
                    .x(x_read),
                    .sv(sv_read),
                    .sv_class(sv_class_read),
                    .coef_scaled(coef_read),
                    .coef_sign(coef_sign_read),
                    .new_computation(new_computation_read),
                    .data_valid(enable_compute),
                    .clk(clk),
                    .reset(reset),
                    .distance(distance_computed)
                );
            `endif
			// Wait for the results of the block
			reg enable_reg_keep [0:`KERNEL_DURATION-1];
			always @(posedge clk) begin
				enable_reg_keep[0] <= reset ? 0 : last_compute;
			end
			for (k = 1; k < `KERNEL_DURATION; k = k + 1) begin: wait_for_keep
				always @(posedge clk) begin
					enable_reg_keep[k] <= reset ? 0 : enable_reg_keep[k-1];
				end
			end
			// Keep the result once total finished
			reg enable_keep;
			always @(posedge clk) begin
				enable_keep <= reset ? 0 : enable_reg_keep[`KERNEL_DURATION-1];
				// The first unrolled loop manages the ready signal
				if (r == 0) begin
					distance_ready_keep <= enable_keep;
				end
			end
			for (d = 0; d < `SVM_DECISION_COUNT; d = d + 1) begin:unpack_distance
				always @(posedge clk) begin
					if (enable_keep)
						distances_keep[r][d] <= distance_computed[(d + 1) * (`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC) - 1 : d * (`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC)];
				end
			end
		end
	endgenerate

	// Sum the computed distances (one per unrolled instance) (sum)
	localparam sum_first_count = `SVM_SV_UNROLL % 2 > 0 ? (`SVM_SV_UNROLL / 2 + 1) : (`SVM_SV_UNROLL / 2);
	reg enable_reg_compare [0:`SVM_SV_UNROLL_LN-1];
	wire enable_compare;
	wire signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] distances_sum [0:`SVM_DECISION_COUNT-1];
	generate
		genvar u, v;
		for (d = 0; d < `SVM_DECISION_COUNT; d = d + 1) begin: add_distance
			// each distance is computed simultaneously
			reg signed [`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC - 1:0] distances_sum_tmp [0:`SVM_SV_UNROLL_LN-1][0:sum_first_count-1];
			for (u = 0; u < `SVM_SV_UNROLL_LN; u = u + 1) begin : add_distances_level
				// 1 level of adders for 1 distance
				for (v = 0; v < (`SVM_SV_UNROLL % (2 ** (u+1)) > 0 ? (`SVM_SV_UNROLL / (2 ** (u+1)) + 1) : (`SVM_SV_UNROLL / (2 ** (u+1)))); v = v + 1) begin : add_distances
					// 1 adder for 1 distance
					always @(posedge clk) begin
						if (u == 0 && (2 * v + 1) < `SVM_SV_UNROLL)
							distances_sum_tmp[u][v] <= distances_keep[2 * v][d] + distances_keep[2 * v + 1][d];
						else if (u == 0)
							distances_sum_tmp[u][v] <= distances_keep[2 * v][d];
						else if (2 * v + 1 < (`SVM_SV_UNROLL % (2 ** u) > 0 ? (`SVM_SV_UNROLL / (2 ** u) + 1) : (`SVM_SV_UNROLL / (2 ** u))))
							distances_sum_tmp[u][v] <= distances_sum_tmp[u-1][2 * v] + distances_sum_tmp[u-1][2 * v + 1];
						else
							distances_sum_tmp[u][v] <= distances_sum_tmp[u-1][2 * v];
					end
					// let the enable follow
					always @(posedge clk) begin
						if (d == 0) begin
							if (u == 0)
								enable_reg_compare[0] <= distance_ready_keep;
							else
								enable_reg_compare[u] <= enable_reg_compare[u-1];
						end
					end
					// get the good distance out of the loops
					if (u == (`SVM_SV_UNROLL_LN - 1) && v == 0)
						assign distances_sum[d] = distances_sum_tmp[u][v];
				end
			end
		end
	endgenerate
	assign enable_compare = enable_reg_compare[`SVM_SV_UNROLL_LN-1];

	// Compare the computed distances with rho values (compare)
	// We assume that they have the same fraction width
	// Initialize rho values at reset: no memory management yet!
	reg signed [`SVM_RHO_WIDTH_INT + `SVM_RHO_WIDTH_FRAC - 1 : 0] rho [0 : `SVM_DECISION_COUNT - 1];
	generate
		genvar dec_id;
		for (dec_id = 0; dec_id < `SVM_DECISION_COUNT; dec_id = dec_id + 1) begin:init_rho
			always @(posedge clk) begin
				if (reset)
					rho[dec_id] <= `SVM_RHO_dec;
				else
					rho[dec_id] <= rho[dec_id];
			end
		end
	endgenerate
	reg enable_vote;
	reg comparisons [0:`SVM_DECISION_COUNT-1];
	generate
		for (d = 0; d < `SVM_DECISION_COUNT; d = d + 1) begin:unpack_distance
			always @(posedge clk) begin
				if (enable_compare)
					comparisons[d] <= distances_sum[d] > rho[d];
				if (d == 0)
					enable_vote <= reset ? 0 : enable_compare;
			end
		end
	endgenerate

	// Compute the number of votes per class (vote)
	wire [`SVM_CLASS_COUNT-2:0] votes_list [0:`SVM_CLASS_COUNT-1];
	generate
		for (dec_id = 0; dec_id < `SVM_DECISION_COUNT; dec_id = dec_id + 1) begin:select_votes
			assign votes_list[`SVM_DEC_SV_CLASS1_dec][`SVM_DEC_OUT_INDEX1_dec] = comparisons[dec_id];
			assign votes_list[`SVM_DEC_SV_CLASS2_dec][`SVM_DEC_OUT_INDEX2_dec] = !comparisons[dec_id];
		end
	endgenerate
	wire [`SVM_CLASS_WIDTH-1:0] votes_sum [0:`SVM_CLASS_COUNT-1];
	reg [`SVM_CLASS_WIDTH-1:0] votes [0:`SVM_CLASS_COUNT-1];
	generate
		for (c = 0; c < `SVM_CLASS_COUNT; c = c + 1) begin:sum_votes
			multi_sum #(
				.VALUE_WIDTH(1),
				.VALUE_COUNT(`SVM_CLASS_COUNT-1),
				.SUM_WIDTH(`SVM_CLASS_WIDTH)
			) square_adder (
				.values(votes_list[c]),
				.sum(votes_sum[c])
			);
			always @(posedge clk) begin
				if (enable_vote)
					votes[c] <= votes_sum[c];
			end
		end
	endgenerate
	reg enable_select;
	always @(posedge clk) begin
		enable_select <= reset ? 0 : enable_vote;
	end

	// Select the class with the most votes (select)
	localparam select_first_count = `SVM_CLASS_COUNT % 2 > 0 ? (`SVM_CLASS_COUNT / 2 + 1) : (`SVM_CLASS_COUNT / 2);
	reg [`SVM_CLASS_WIDTH-1:0] votes_tmp [0:`SVM_CLASS_COUNT_LN-1][0:select_first_count-1];
	reg [`SVM_CLASS_WIDTH-1:0] classes_tmp [0:`SVM_CLASS_COUNT_LN-1][0:select_first_count-1];
	reg enable_reg_out [0:`SVM_CLASS_COUNT_LN-1];
	generate
		for (u = 0; u < `SVM_CLASS_COUNT_LN; u = u + 1) begin : max_level
			// 1 level of comparators (simultaneous computation)
			for (v = 0; v < (`SVM_CLASS_COUNT % (2 ** (u+1)) > 0 ? (`SVM_CLASS_COUNT / (2 ** (u+1)) + 1) : (`SVM_CLASS_COUNT / (2 ** (u+1)))); v = v + 1) begin : max_instance
				// 1 comparator
				always @(posedge clk) begin
					if (u == 0) begin
						if ((2 * v + 1) < `SVM_CLASS_COUNT) begin
							// Checks on 2*v+1 are useless, except to avoid synthesis errors
							votes_tmp[u][v] <= (votes[(2 * v + 1) < `SVM_CLASS_COUNT ? (2 * v + 1) : 0] > votes[2 * v]) ? votes[(2 * v + 1) < `SVM_CLASS_COUNT ? (2 * v + 1) : 0] : votes[2 * v];
							classes_tmp[u][v] <= (votes[(2 * v + 1) < `SVM_CLASS_COUNT ? (2 * v + 1) : 0] > votes[2 * v]) ? (2 * v + 1) : (2 * v);
						end else begin
							votes_tmp[u][v] <= votes[2 * v];
							classes_tmp[u][v] <= (2 * v);
						end
					end else begin
						// Checks on u-1 are useless, except to avoid synthesis errors
						if ((2 * v + 1) < (`SVM_CLASS_COUNT % (2 ** u) > 0 ? (`SVM_CLASS_COUNT / (2 ** u) + 1) : (`SVM_CLASS_COUNT / (2 ** u)))) begin
							// Checks on 2*v+1 are useless, except to avoid synthesis errors
							votes_tmp[u][v] <= (votes_tmp[u == 0 ? 0 : (u-1)][(2 * v + 1) < (`SVM_CLASS_COUNT % (2 ** u) > 0 ? (`SVM_CLASS_COUNT / (2 ** u) + 1) : (`SVM_CLASS_COUNT / (2 ** u))) ? (2 * v + 1) : 0] > votes_tmp[u == 0 ? 0 : (u-1)][2 * v]) ? votes_tmp[u == 0 ? 0 : (u-1)][(2 * v + 1) < (`SVM_CLASS_COUNT % (2 ** u) > 0 ? (`SVM_CLASS_COUNT / (2 ** u) + 1) : (`SVM_CLASS_COUNT / (2 ** u))) ? (2 * v + 1) : 0] : votes_tmp[u == 0 ? 0 : (u-1)][2 * v];
							classes_tmp[u][v] <= (votes_tmp[u == 0 ? 0 : (u-1)][(2 * v + 1) < (`SVM_CLASS_COUNT % (2 ** u) > 0 ? (`SVM_CLASS_COUNT / (2 ** u) + 1) : (`SVM_CLASS_COUNT / (2 ** u))) ? (2 * v + 1) : 0] > votes_tmp[u == 0 ? 0 : (u-1)][2 * v]) ? classes_tmp[u == 0 ? 0 : (u-1)][(2 * v + 1) < (`SVM_CLASS_COUNT % (2 ** u) > 0 ? (`SVM_CLASS_COUNT / (2 ** u) + 1) : (`SVM_CLASS_COUNT / (2 ** u))) ? (2 * v + 1) : 0] : classes_tmp[u == 0 ? 0 : (u-1)][2 * v];
						end else begin
							votes_tmp[u][v] <= votes_tmp[u-1][2 * v];
							classes_tmp[u][v] <= classes_tmp[u-1][2 * v];
						end
					end
				end
			end
			// let the enable follow
			always @(posedge clk) begin
				if (u == 0)
					enable_reg_out[0] <= reset ? 0 : enable_select;
				else
					enable_reg_out[u] <= reset ? 0 : enable_reg_out[u-1];
			end
		end
	endgenerate
	assign new_result = enable_reg_out[`SVM_CLASS_COUNT_LN-1];
	assign class = classes_tmp[`SVM_CLASS_COUNT_LN-1][0];
endmodule
