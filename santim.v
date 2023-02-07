module santim(
   input       clock,      // 2,5 MHz тактовая
   input       rst,        // Сигнал сброса
	input			pwse,			// Таймер разрешен при включении
	input [2:0]	sanity,		// Значение задержки
	input			ena,			// Разрешение работы
   output		out			// Выход, активный 1
);

assign out = nbdcok;

reg  [6:0]	sanity_cnt;		// Таймер
wire 			qsc;				// Тактовая 1/4 секунды
wire 			mc;				// Тактовая 1 мин.

reg			nbdcok;
reg  [3:0]	bdcok_cnt;
localparam	BDCOK_LIMIT = 9;

// Генерация тактовой 1/4 секунды
time_counter #(.LIMIT(1000000)) qsecs
(
	.clock(clock),
	.rst(rst),
	.tc(qsc)
);

/*
wire sc;
time_counter #(.LIMIT(4)) secs
(
	.clock(qsc),
	.rst(rst),
	.tc(sc)
);

time_counter #(.LIMIT(60)) mins
(
	.clock(sc),
	.rst(rst),
	.tc(mc)
);
*/

// Генерация тактовой 1 минута
time_counter #(.LIMIT(240)) mins
(
	.clock(qsc),
	.rst(rst),
	.tc(mc)
);

wire sanity_clk = sanity[2]? mc : qsc;		// Новая тактовая
wire nzc = |sanity_cnt;							// сигнал "таймер не нуль"
wire timer_ena = pwse | ena;					// Разрешение работы таймера
wire reset= rst | (~ena);
always @(posedge sanity_clk, posedge reset) begin
	if(reset) begin
		casez(sanity) // Началаьное значение таймера
			3'b?00: sanity_cnt = 7'd1;			// 1
			3'b?01: sanity_cnt = 7'd1;			// 4
			3'b?10: sanity_cnt = 7'd4;			// 16
			3'b?11: sanity_cnt = 7'd16;		// 64
		endcase
	end
	else begin
		if(timer_ena) sanity_cnt <= sanity_cnt - 1'b1;
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

always @(posedge clock, posedge rst) begin
   if(rst) begin
      delay <= 0; tics <= 1'b0;
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
