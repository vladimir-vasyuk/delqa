//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль MDC
//=================================================================================
module mdc(
	input  	   	clock,		// Синхросигнал, мин. период 400 нс
	input				rst,			// Сигнал сброса
	input				evt,			// Сигнал периодического опроса состояния
	inout				mdiol,		// Линия данных
	output reg		err,			// Сигнал ошибки (не задействован)
	input  [6:0]	ctrl,			// Контрольный байт
	input  [15:0]	val,			// Данные записи в регистры
	output [15:0]	out,			// Данные чтения из регистров
	output [7:0]	status		// Статусные биты
);

wire [4:0]  phy = 5'b00001;	// Адрес интерфейса
wire [15:0] datai;				// Шина входных данных
wire [15:0] datao;				// Шина выходных данных
reg 			start;				// Сигнал начала операции
reg			rw;					// Код операции (0-чтение, 1-запись)
wire			done;					// Сигнал завершения операции
reg  [4:0]  mdc_addr;			// Регистр адреса
reg  [15:0]	datout;				// Данные чтения из регистров
assign out[15:0] = datout[15:0];
assign datai[15:0] = val[15:0];

// Status regs
reg			rdy;
reg  [1:0]	speed;				// Скорость:	10-1000; 01-100; 00-10; 11-зарезервированно
reg			duplex;				// Дуплекс:		1-полный; 0-полудуплекс
reg			mdi;					// Интерфейс:	1-MDI crossover; 0-MDI
reg			lrec;					// Приемник:	1-готов; 0-не готов
reg			link;					// Связь:		1-есть; 0-нет
assign status = {rdy, speed, duplex, 1'b0, mdi, lrec, link};

initial begin
	proc_status <= 1'b0;
	start <= 1'b0;
	rw <= 1'b0;
	state <= IDLE;
	err <= 1'b0;
end

//////////////MDC state machine///////////////
localparam	IDLE	= 3'b000;
localparam	READ	= 3'b001;
localparam	WRITE	= 3'b010;
localparam	WAIT	= 3'b011;
localparam	GETST	= 3'b100;
localparam	COPY	= 3'b101;
reg [2:0] state;
reg proc_status;

always @(posedge clock or posedge rst) begin
//always @(posedge clock) begin
	if(rst) begin
		start <= 1'b0; rw <= 1'b0;
		proc_status <= 1'b0;
//		err <= 1'b0;
		state <= IDLE;
	end
	else begin
		case(state)
			IDLE: begin
				rdy <= 1'b1;							// Установить сигнал готовности
				if(~done & ~start) begin			// Модуль не занят?
					if(evt) begin						// Получен сигнал периодического опроса
						rdy <= 1'b0;					// Да - сброс сигнала готовности ...
						proc_status <= 1'b1;			// ... установить сигнал периодической обработки ...
						mdc_addr <= 5'h11;			// ... регистр состояния ...
						state <= READ;					// ... переход к операции чтения
					end
					else begin
						if(ctrl[5]) begin						// Установлен бит операции? 
							if(ctrl[6]) state <= WRITE;	// операция записи ...
							else			state <= READ;		// ... или операция чтения ...
							mdc_addr <= ctrl[4:0];			// ... получить номер регистра ...
							rdy <= 1'b0;						// ... сброс сигнала готовности
						end
					end
				end
			end
			READ: begin									// Операция чтения регистра
				rw <= 1'b0;								// Код операции чтения
				start <= 1'b1;							// Сигнал работы
				state <= WAIT;							// Переход к подтверждению
			end
			WRITE: begin								// Операция записи регистра
				rw <= 1'b1; 							// Код операции записи
				start <= 1'b1;							// Сигнал работы
				state <= WAIT;							// Переход к подтверждению
			end
			WAIT: begin
				if(done) begin							// Получен сигнал подтверждения
					start <= 1'b0;						// Сброс сигнала работы
					if(proc_status)					// Установлен сигнал периодической обработки?
						state <= GETST;				// Да - переход к формированию статусного регистра
					else
						state <= COPY;					// Нет - завершение
				end
			end
			GETST: begin								//  Формирование статусного регистра
				speed[1:0] <= datao[15:14];		// Скорость
				duplex <= datao[13];					// Дуплекс
				link <= datao[10];					// Связь
				mdi <= datao[6];						// Тип
				lrec <= datao[1];						// Приемник
				proc_status <= 1'b0;					// Сброс сигнала периодической обработки
				state <= IDLE;							// Переход в режим ожидания
			end
			COPY:	begin
				if(~ctrl[6]) datout <= datao;		// Если была операция чтения - данные на выход
				state <= IDLE;							// Переход в режим ожидания
			end
		endcase
	end
end
//////////////////////////////////////////////

mdio mdiom(
	.mdc(clock),
	.rst(rst),
	.phy_addr(phy),
	.reg_addr(mdc_addr),
	.data_i(datai),
	.data_o(datao),
	.start(start),
	.rw(rw),
	.mdiol(mdiol),
	.done(done)
);         

endmodule
