module mdc_clk(
   input       clock,      // 50 MHz clock
   input       rst,        // Reset signal
   output reg  mdcclk,     // MD clock (T=440ns)
   output reg  mdsevt      // MD event (T~1.85sec)
);

reg [4:0] delay = 5'o0;
reg [21:0] sdelay = 22'b0;
localparam limit = 5'd9;

always @(posedge clock or posedge rst) begin
   if(rst) begin
      delay <= 5'o0; mdcclk <= 1'b0;
   end
   else begin
      if(delay == limit) begin
         delay <= 5'o0;
         mdcclk <= ~mdcclk;
      end
      else delay <= delay + 1'b1;
   end
end

always @(posedge mdcclk or posedge rst) begin
   if(rst) begin
      sdelay <= 22'o0; mdsevt <= 1'b0;
   end
   else begin
      if(&sdelay) mdsevt <= 1'b1;
      else			mdsevt <= 1'b0;
      sdelay <= sdelay + 1'b1;
   end
end

endmodule
