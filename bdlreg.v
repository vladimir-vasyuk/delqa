//=================================================================================
// Реализация контроллера Ethernet DELQA
//---------------------------------------------------------------------------------
// Регистры BDL
//=================================================================================
module bdlreg #(parameter NUM=6)
(
   input                clk,
   input  [NUM/2-1:0]   addr,
   input  [15:0]        data,
   input                we,
   output [15:0]        q
);

reg[15:0] x[NUM-1:0];

initial begin
	x[0] <= 16'hFFFF;
end

assign q = x[addr];

always @(posedge clk) begin
   if(we) begin
      x[addr] <= data;
   end
end

endmodule
