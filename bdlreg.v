//=================================================================================
// Реализация контроллера Ethernet DELQA
//---------------------------------------------------------------------------------
// Регистры BDL
//=================================================================================
module bdlreg #(parameter NUM=4)
(        
    input 					clk,
    input [NUM/2-1:0]	addr,
    input [15:0]			data,
    input 					we,
    output [15:0] 		q
);

reg[15:0] x[NUM-1:0];

genvar i;
generate
for (i = 0; i < NUM; i = i + 1)
begin : reg_init
    initial
        x[i] = 16'b0; // обнуление регистров
end
endgenerate

assign q = x[addr];

always @(posedge clk) begin
	if(we) begin
		x[addr] <= data;
	end
end

endmodule
