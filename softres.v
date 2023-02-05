module soft_reset(
	input			clk,
	input			rst,
	input			csr_sr,
	output		block,
	output		reset
);

reg  [1:0]	reset_r;
wire combrst = rst | reset_r[1];
assign reset = reset_r[1];
assign block = reset_r[0];

always @(posedge clk) begin
	if(combrst)
		reset_r <= 2'b0;
	else begin
		if(csr_sr)
			reset_r[0] <= 1'b1;
		else begin
			if(reset_r[0])
				reset_r[1] <= 1'b1;
		end
	end
end

endmodule
