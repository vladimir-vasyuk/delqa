module santim(
   input       clock,      // 2,5 MHz тактовая
   input       rst,        // Сигнал сброса
	input [2:0]	sanity,		// Значение задержки
	input			ena,			// Разрешение работы
   output		out			// Выход, активный 1
);

assign out = ~nbdcok;

reg  [6:0]	sanity_cnt;		// Таймер
wire 			qsc;				// Тактовая 1/4 секунды
wire 			mc;				// Тактовая 1 мин.

reg			nbdcok;
reg  [3:0]	bdcok_cnt;
localparam	BDCOK_LIMIT = 10;

// Генерация тактовой 1/4 секунды
time_counter #(.LIMIT(312500)) qsecs
(
	.clock(clock),
	.rst(rst),
	.tc(qsc)
);

// Генерация тактовой 1 минута
time_counter #(.LIMIT(120)) mins
(
	.clock(qsc),
	.rst(rst),
	.tc(mc)
);

wire sanity_clk = sanity[2]? mc : qsc;		// Новая тактовая
wire nzc = |sanity_cnt;							// Сигнал "таймер не нуль"
wire reset= rst | (~ena);						// Комбинированный сброс 
always @(posedge sanity_clk, posedge reset) begin
	if(reset) begin
		casez(sanity) // Началаьное значение таймера
			3'b?00: sanity_cnt = 7'o177;		// -1
			3'b?01: sanity_cnt = 7'o174;		// -4
			3'b?10: sanity_cnt = 7'o160;		// -16
			3'b?11: sanity_cnt = 7'o100;		// -64
		endcase
	end
	else begin
		if(ena & nzc) sanity_cnt <= sanity_cnt + 1'b1;
	end
end

// Генерация отрицательного BDCOK ~4 msec.
always @(posedge clock) begin
	if(nzc) begin
		nbdcok <= 1'b1; bdcok_cnt <= 4'b0;
	end
	else begin
		if(bdcok_cnt != BDCOK_LIMIT) begin
			bdcok_cnt <= bdcok_cnt + 1'b1;
			nbdcok <= 1'b0;
		end
		else
			nbdcok <= 1'b1;
	end
end

endmodule


module time_counter #(parameter LIMIT=60)
(
	input		clock,
	input		rst,
	output	tc
);

assign tc = tics;

localparam	DREG_WIDTH = log2(LIMIT);

reg  [DREG_WIDTH-1:0]	delay;
reg							tics;

always @(posedge clock) begin
   if(rst) begin
      delay <= 0; tics <= 1'b1;
   end
   else begin
		if(delay == (LIMIT-1)) begin
			tics <= ~tics;
			delay <= 0;
		end
		else
			delay <= delay + 1'b1;
   end
end

function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1)
         value = value >> 1;
   end
endfunction

endmodule
