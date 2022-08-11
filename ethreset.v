//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Формирование сигнала сброса
//=================================================================================
module ethreset(
	input		clk,			// 50МГц тактовая
	input		rst,			// Сигнал сброса (кнопка, программный и т.д.)
	output	e_reset		// Сигнал сброса для Ethernet (~10.5мсек)
);

reg [19:0]	to;
reg			reset;
wire			start;
assign e_reset = reset;

initial begin
	reset <= 1'b0;
end

// Формирование сигнала старта по переднему фронту
reg prev_sig;
always @(posedge clk)
	prev_sig <= rst;
assign start = ~prev_sig & rst;

// Формирование сигнала сброса
always @(posedge clk) begin
	if(start) reset <= 1'b1;
	else if(to[19] & to[17]) reset <= 1'b0;
end

// Счет с обнудением
always @(posedge clk, posedge start) begin
	if(start) to <= 20'b0;
	else		 to <= to + 1'b1;
end

endmodule
