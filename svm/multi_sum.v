// Compute a sum of multiple values
// The sum computations are parallelized

`timescale 1ns / 1ps

module multi_sum
#(
	parameter VALUE_WIDTH = 8,
	parameter VALUE_COUNT = 2,
	parameter SUM_WIDTH = 9
)
(
    input [VALUE_WIDTH * VALUE_COUNT - 1:0] values,
    output [SUM_WIDTH - 1:0] sum
);

   function integer log2;
      input integer number;
      begin
         log2=0;
         while(2**log2<number) begin
            log2=log2+1;
         end
      end
   endfunction
	
	function integer sum_count;
		input integer count;
		input integer level;
		begin
			if (count % (2 ** level) > 0) begin
				sum_count = count / (2 ** level) + 1;
			end else begin
				sum_count = count / (2 ** level);
			end
		end
	endfunction

	wire [SUM_WIDTH-1:0] sum_tmp [0:VALUE_COUNT-1][0:VALUE_COUNT-1];
	
	// 2 input adders
	generate
		genvar j, k;
		for (j = 0; j <= log2(VALUE_COUNT); j = j + 1) begin : add_level
			// 1 level of adders (simultaneous computation)
			for (k = 0; k < sum_count(VALUE_COUNT, j); k = k + 1) begin : add_item
				// 1 adder
				if (j == 0)
					assign sum_tmp[0][k] = values[k * VALUE_WIDTH + VALUE_WIDTH - 1: k * VALUE_WIDTH];
				else if (2 * k + 1 < sum_count(VALUE_COUNT, j-1))
					assign sum_tmp[j][k] = sum_tmp[j-1][2 * k] + sum_tmp[j-1][2 * k + 1];
				else
					assign sum_tmp[j][k] = sum_tmp[j-1][2 * k];
			end
		end
	endgenerate
	
	// Final value to output
	assign sum = sum_tmp[log2(VALUE_COUNT)][0];
endmodule
