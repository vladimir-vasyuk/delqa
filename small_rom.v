module small_rom
#(parameter ADDR_WIDTH = 3, parameter WIDTH = 8, parameter FILE = "sarom.txt")
(
   input                      clk,
// input [ADDR_WIDTH - 1:0]   addr,
// output [WIDTH - 1:0]    	q
   output [63:0]              q
);

reg [WIDTH - 1:0]mem[2**ADDR_WIDTH - 1:0];

//assign q = mem[addr];
assign q = {mem[7], mem[6], mem[5], mem[4], mem[3], mem[2], mem[1], mem[0]};

initial begin
   $readmemh(FILE, mem);
end

endmodule
