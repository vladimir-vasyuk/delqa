//=================================================================================
// Реализация контроллера Ethernet DELQA
//
// Что нужно сделать:
// 1. Разобраться с вычислением контрольной суммы MAC адреса - сделано.
// 2. Разобраться с разделением режимов чтения пакета и чтения ROM - сделано (для ROM не полноценный RBDL).
// 3. Разобраться с контрольной суммой ROM - сделано.
// 4. Изменить логику (разделить запись и чтение, обеспечить работу loopback) - сделано.
// 5. Обеспечить функционал Setup packets (фильрация и режимы) - 0%.
// !!! Обязаловка CRC приводит к неверным данным о длине пакета
//
//=================================================================================
module delqa (
   input						wb_clk_i,   // тактовая частота шины
   input						wb_rst_i,   // сброс
   input      [3:0]		wb_adr_i,   // адрес 
   input      [15:0]		wb_dat_i,   // входные данные
   output reg [15:0]		wb_dat_o,   // выходные данные
   input						wb_cyc_i,   // начало цикла шины
   input						wb_we_i,    // разрешение записи (0 - чтение)
   input						wb_stb_i,   // строб цикла шины
   input      [1:0]		wb_sel_i,   // выбор байтов для записи 
   output					wb_ack_o,   // подтверждение выбора устройства
	output					nbdcok,

// обработка прерывания   
   output reg           irq,        // запрос
   input                iack,       // подтверждение

// DMA
   output               dma_req,    // запрос DMA
   input                dma_gnt,    // подтверждение DMA
   output    [21:0]		dma_adr_o,  // выходной адрес при DMA-обмене
   input     [15:0]		dma_dat_i,  // входная шина данных DMA
   output    [15:0]		dma_dat_o,  // выходная шина данных DMA
   output 					dma_stb_o,  // строб цикла шины DMA
   output 					dma_we_o,   // направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
   input						dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен

// интерфейс ethernet шины
   input						e_rxc,      // Receive clock
   input						e_rxdv,     // Receive data valid
   input						e_rxer,     // Receive error
   input     [7:0]		e_rxd,      // Receive data
   input						e_crs,      // Carrier sense
   input						e_txc,      // Transmit clock
   output					e_txen,     // Tramsmit enable
   output					e_txer,     // Tramsmit error
   output    [7:0]		e_txd,      // Transmit data
   output					e_rst,      // Hardware reset, active low
   output					e_mdc,      // MDC clock
   inout						e_mdio,     // MD line
   output					e_gtxc,		// 125 MHz

// переключатели
	input						s3,			// выбор режима
	input						s4				// выбор дополнений
);

//************************************************
// Сигналы упраления обменом с шиной
//************************************************
wire        bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;   // строб цикла шины
wire        bus_read_req = bus_strobe & ~wb_we_i;			   // запрос чтения
wire        bus_write_req = bus_strobe & wb_we_i;			   // запрос записи

//************************************************
// регистры контроллера DMA
//************************************************
wire        nxm;              // признак таймаута шины
reg         dmawr;            // запуск записи
reg         dmard;            // запуск чтения
wire        dmacomplete;      // признак завершения работы DMA-контроллера
reg  [11:0] wcount;
reg  [12:0]	bcount;
wire [11:0] swcount = {4'b0,bcount[8:7], 6'b0};

//************************************************
// Переключатели
//************************************************
reg         s1;               // выбор адреса устройства
//reg			s3;               // выбор режима
//reg			s4;               // выбор дополнений
wire pwse = s3 & s4;				// Sanity Timer разрешен при включении

// Комбинированный сброс
wire			res_soft;
wire        comb_res = res_soft | wb_rst_i;

reg			busy;					// Регистр индикации занятости
reg			st_ena;				// Разрешение/запрет для sanity timer
reg			st_res;				// Сброс sanity timer нормальной операцией

//************************************************
// Base address 174440 (17774440 - 22-битная шина)
//************************************************
// Чтение:  00 - mac address byte / MD state
//          02 - mac address byte / MD reg. errors
//          04 - mac address byte / MD reg. low byte
//          06 - mac address byte / MD reg. high byte
//          10 - mac address byte
//          12 - mac address byte
//          14 - vector address register
//          16 - csr
// Запись:  00 - [15:8] MD control byte
//          02 - MD reg. value
//          04 - receive BDL start address register (low)
//          06 - receive BDL start address register (high)
//          10 - transmit BDL start address register (low)
//          12 - transmit BDL start address register (high)
//          14 - vector address register
//          16 - csr

//************************************************
// Регистр управления/состояния - csr - 174456
//************************************************
reg         csr_ri = 1'b0;    // 15	Receive Interrupt Request (RW)
//reg       csr_pe = 1'b0;    // 14	Parity Error in Memory (RO)
wire        csr_ca;           // 13	Carrier from Receiver Enabled (RO)
//reg       csr_ok = 1'b1;    // 12	Ethernet Transceiver Power OK (RO)
//reg       csr_rr = 1'b0;    // 11	reserved
reg         csr_se = 1'b0;    // 10	Sanity Timer Enable (RW)
reg         csr_el = 1'b0;    // 09	External  Loopback (RW)
reg         csr_il = 1'b0;    // 08	Internal Loopback (RW) active low
reg         csr_xi = 1'b0;    // 07	Transmit Interrupt Request (RW1)
reg         csr_ie = 1'b0;    // 06	Interrupt Enable (RW)
reg         csr_rl = 1'b1;    // 05	Receive List Invalid/Empty (RO)
reg         csr_xl = 1'b1;    // 04	Transmit List Invalid/Empty (RO)
reg         csr_bd = 1'b0;    // 03	Boot/Diagnostic ROM load (RW)
reg         csr_ni = 1'b0;    // 02	Nonexistance-memory timeout Interrupt (RO)
reg         csr_sr = 1'b0;    // 01	Software Reset (RW)
reg         csr_re = 1'b0;    // 00	Receiver Enable (RW)
wire [15:0] csr;
assign csr_ca = (~csr_il)? 1'b0 : (~errs[4]);
assign csr = {csr_ri,1'b0,1'b0,1'b1,1'b0,csr_se,csr_el,csr_il,csr_xi,csr_ie,csr_rl,csr_xl,csr_bd,csr_ni,csr_sr,csr_re};


//************************************************
// Регистр адреса ветора - var - 174454
//************************************************
reg         var_ms;           // Mode select (RW) (After power-up reset reflect s3)
reg         var_os;           // Option switch (s4) settings (RO) (After power-up reset reflect s4)
reg         var_rs = 1'b1;    // Request self-test (RW)
reg         var_s3 = 1'b1;    // Self test status (RO)
reg         var_s2 = 1'b1;    // Self test status (RO)
reg         var_s1 = 1'b1;    // Self test status (RO)
reg  [7:0]  var_iv;           // Interrupt vector
//reg       var_rr;           //
reg         var_id = 1'b0;    // Identity test bit
wire [15:0] vareg;
assign vareg = {var_ms,var_os,var_rs,var_s3,var_s2,var_s1,var_iv,1'b0,var_id};

//************************************************
// Регистр адреса блока приема (RBDL) - 174444, 174446
//************************************************
reg  [15:1] rbdl_lwr;         // low address bits
reg  [5:0]  rbdl_hir;         // high address bits
reg         rdstart = 1'b0;   // Флаг функции
reg         romstart = 1'b0;  // Флаг функции
reg			rbdli = 1'b0;

//************************************************
// Регистр адреса блока передачи (TBDL) - 174450, 174452
//************************************************
reg  [15:1] tbdl_lwr;         // low address bits
reg  [5:0]  tbdl_hir;         // high address bits
reg  [21:1] haddr;            // Физичесикй адрес памяти
reg         txstart = 1'b0;   // Флаг функции

reg  [15:1] haddr_sw;			// Копия младших разрядов (для передачи статусных слов)
reg  			bdlc;
reg			tbdli = 1'b0;

//************************************************
// Регистр состояния блока MD (временное решение) - 174440
//************************************************
wire [7:0]  md_status;        // 7: 1/0 - ready/busy
                              // 6,5: 10-1000Мб/с; 01-100Мб/с; 00-10Мб/с; 11-зарезервированно
                              // 4: 1-полный дуплекс; 0-полудуплекс
                              // 3: зарезервированно (0)
                              // 2: 1-MDI crossover; 0-MDI
                              // 1: 1-приемник готов; 0-приемник не готов
                              // 0: 1-связь есть; 0-связи нет

//************************************************
// Регистр записи данных блока MD (временное решение) - 174442
//************************************************
reg  [15:0] md_val;
// Регистр чтения данных блока MD (временное решение) - 174444 -174446
wire [15:0] md_out;

//************************************************
// Регистр записи данных блока MD (временное решение) - 174440
//************************************************
reg         md_func;          //	1/0 - write/read
reg         md_cmd;           // 1 - start
reg  [4:0]  md_reg;           // reg. address
wire [6:0]  md_ctrl = {md_func,md_cmd,md_reg};
reg         md_proc;
reg         md_mux;           // MD мультиплексер (1/0 - On/Off)

//************************************************
// Регистры контроля операции чтения/записи 
//************************************************
reg			rx_rdy;					// Регистр готовности режима приема
reg         txproc, txerr;			// Флаги режима передачи
reg         rdproc, rderr;			// Флаги режима приема

//************************************************
// Регистры BDL descriptor bits
//************************************************
reg         bdl_v, bdl_c, bdl_e, bdl_s, bdl_l, bdl_h; // BDL descriptor bits
reg			setupreg = 1'b0;
reg  [3:0]  bdl_rez;

//************************************************
// Loopback modes
//************************************************
wire        intmode = (~csr_il) & (~csr_el); // Internal loopback
wire        intextmode = (~csr_il) & csr_el; // Internal extended loopback
wire        extmode = csr_il & csr_el;       // External loopback
wire        rxmode = csr_re & (~csr_rl);     // Разрешение приема пакета
wire        loop = (intmode | intextmode | setupreg) & ~csr_re;

//************************************************
// Status words bits
//************************************************
wire			st_esetup = extmode | intextmode | setupreg;
wire			st_runt = (rderr | txerr) & intmode;
reg  [10:0] st_rbl;									// Счетчик байтов приема (коректированный)
wire			st_crcerr;
wire			st_frame;
wire			st_txerr;
wire			st_rxerr;
assign st_crcerr = errs[1];
assign st_frame = errs[1];
assign st_txerr = errs[2] | errs[4];
assign st_rxerr = errs[0] | errs[1] | errs[4];

//************************************************
// Конечный автомат обработки прерывания
//************************************************
localparam[1:0]   i_idle = 0;		// ожидание прерывания
localparam[1:0]   i_req  = 1;		// запрос векторного прерывания
localparam[1:0]   i_wait = 2;		// ожидание обработки прерывания со стороны процессора
reg  [1:0]  interrupt_state;
reg         int_req;

//************************************************
// Конечный автомат обработки функций 1
//************************************************
localparam[4:0]   fp_idle = 0;
localparam[4:0]   fp_bdl0 = 1;
localparam[4:0]   fp_bdl1 = 2;
localparam[4:0]   fp_bdl2 = 3;
localparam[4:0]   fp_bdl3 = 4;
localparam[4:0]   fp_bdl4 = 5;
localparam[4:0]   fp_bdl5 = 6;
localparam[4:0]   fp_bdls = 7;
localparam[4:0]   fp_bdlc = 8;
localparam[4:0]   fp_bdle = 9;
localparam[4:0]   fp_txp0 = 10;
localparam[4:0]   fp_txp1 = 11;
localparam[4:0]   fp_etxp = 12;
localparam[4:0]   fp_ftxp = 13;
localparam[4:0]   fp_tbdl = 14;
localparam[4:0]   fp_rdp0 = 15;
localparam[4:0]   fp_rdp1 = 16;
localparam[4:0]   fp_rlng = 17;
localparam[4:0]   fp_erdp = 18;
localparam[4:0]   fp_frdp = 19;
localparam[4:0]   fp_rbdl = 20;
localparam[4:0]   fp_stw0 = 21;
localparam[4:0]   fp_stw1 = 22;
localparam[4:0]   fp_stwa = 23;
localparam[4:0]   fp_stw2 = 24;
localparam[4:0]   fp_stwh = 25;
localparam[4:0]   fp_rom1 = 26;
localparam[4:0]   fp_rom2 = 27;
reg  [4:0]  fp_state;
reg  [4:0]  fp_next;
wire [3:0]  fp_mode = {1'b00, romstart, rdstart, txstart};


//************************************************
// Конечный автомат обработки функци 2
//************************************************
localparam[2:0]   f_idl = 0;  // холостой ход
//localparam[2:0]   f_rst = 1;  // программный сброс
//localparam[2:0]   f_txp = 2;  // передача сетевого пакета
localparam[2:0]   f_mdc = 1;  // MD чтение/запись
localparam[2:0]   f_st  = 2;  // Self-test
localparam[2:0]   f_bdr = 3;  // Load boot ROM
reg  [2:0]  funcreg;

//************************************************
//* Блок памяти
//************************************************
reg  [10:0] baddr;      // Регистр адреса (начальное занчение)
wire [10:0] baddrinc;   // Регистр адреса - инкремент при DMA/ПДП 
wire [8:0]  etxaddr;    // Регистр адреса блока передачи (память -> ether модуль)
wire [8:0]  erxaddr;    // Регистр адреса блока приема (ether модуль -> память)
//
wire [31:0] etxdbus;    // Шина данных блока передачи (память -> ether модуль)
wire [15:0] mtxdbus;    // Шина данных блока приема (DMA -> память)
wire [15:0] mrxdbus;    // Шина данных блока приема (память -> DMA)
wire [31:0] erxdbus;    // Шина данных блока приема (ether модуль -> память)
//
wire        mtxwe;
wire        erxwe;
wire			dma_we = dma_ack_i & dma_gnt;
assign mtxwe = (dmawr & (fp_state == fp_txp1))? dma_we : 1'b0;

e1632bm txmem(
   .data(mtxdbus),
   .rdaddress(etxaddr),
   .rdclock(e_rxc),
   .wraddress(baddrinc[9:0]),
   .wrclock(wb_clk_i),
   .wren(mtxwe),
   .q(etxdbus)
);

e3216bm rxmem(
   .data(erxdbus),
   .rdaddress(baddrinc[9:0]),
   .rdclock(wb_clk_i),
   .wraddress(erxaddr),
   .wrclock(rxclkb),
   .wren(erxwe),
   .q(mrxdbus)
);

//************************************************
// Регистры BDL
//************************************************
wire [15:0] bdldat;
reg         locaddr = 1'b0;
reg			bdlbus = 1'b0;
reg         stwwe = 1'b0;
reg  [15:0] stwdat;
wire [2:0]  baddrloc = locaddr? baddr[2:0] : baddrinc[2:0];
wire			dmabdl = (dmawr & (fp_state == fp_bdl1))? 1'b1 : 1'b0;
wire			bdlwe = dmabdl? dma_we : stwwe;
wire [15:0]	bdldin = dmabdl? mtxdbus : stwdat;

bdlreg #(.NUM(6)) bdl(
   .clk(wb_clk_i),
   .addr(baddrloc[2:0]),
   .data(bdldin),
   .we(bdlwe),
   .q(bdldat)
);

//************************************************
// BD ROM ROM
//************************************************
wire [15:0] romdat;
rom bdrom(
   .address(baddrinc),
   .clock(wb_clk_i),
   .q(romdat)
);

//************************************************
// MAC address ROM
//************************************************
wire        sa_rom_chk;       // Checksum signal
wire [63:0] macval;
reg  [2:0]  maddr;
assign sa_rom_chk = csr_el & (~csr_bd) & (~csr_re);
small_rom sarom(
   .clk(wb_clk_i),
//	 .addr(maddr),
   .q(macval)
);

//************************************************
// Ethernet module
//************************************************
reg			mcasf;
reg			promf;
reg  [2:0]	sanity;
reg         rxdone;     // Флаг завершения приема
wire        rxrdyw;     // Флаг готовности приема
wire        rxclkb;     // Синхросигнал канала приема (запись в буферную память)
wire        txdonew;    // Флаг завершения передачи
reg         txrdy;      // Флаг готовности передачи
reg  [10:0] txcntb;     // Счетчик байтов передачи
wire [10:0] rxcntb;     // Счетчик байтов приема
wire [4:0]  lbmode;     // Режим работы модуля Ethernet
wire [7:0]  errs;       // Ошибки приема/передачи
assign lbmode = {bdl_s, bdl_h, extmode, loop, rxmode};

wire ereset;
ethreset ethrstm(
   .clk(wb_clk_i),
   .rst(comb_res),
   .e_reset(ereset)
);

ether etherm(
   .rst(ereset),
	.mcast(mcasf),
	.promis(promf),
   .txrdy(txrdy),
   .rxdone(rxdone),
   .txcntb(txcntb),
   .etxdbus(etxdbus),
   .etxaddr(etxaddr),
   .erxaddr(erxaddr),
   .erxdbus(erxdbus),
   .rxcntb(rxcntb),
   .erxwrn(erxwe),
   .rxclkb(rxclkb),
   .rxrdy(rxrdyw),
   .txdone(txdonew),
   .lbmode(lbmode),
   .errs(errs),
// .macbus(mymac),
   .e_rxc(e_rxc),
   .e_rxdv(e_rxdv),
   .e_rxer(e_rxer),
   .e_rxd(e_rxd),
   .e_crs(e_crs),
   .e_txc(e_txc),
   .e_txen(e_txen),
   .e_txer(e_txer),
   .e_txd(e_txd),
   .e_rst(e_rst),
   .e_gtxc(e_gtxc),
   .md_clk(md_clock),
   .md_evt(md_evt),
   .e_mdio(e_mdio),
   .md_ctrl(md_ctrl),
   .md_val(md_val),
   .md_out(md_out),
   .md_status(md_status)
);

wire	blkbus;
wire	allow_bus_ops = ~(blkbus | busy);

soft_reset sftresm(
	.clk(wb_clk_i),
	.rst(wb_rst_i),
	.csr_sr(csr_sr),
	.block(blkbus),
	.reset(res_soft)
);

//************************************************
// Инициализация
//************************************************
initial begin
   s1 <= 1'b0; //s3 <= 1'b0; s4 <= 1'b0;
   funcreg <= 3'b0; 
   dmawr <= 1'b0; dmard <= 1'b0;
   txrdy <= 1'b0; rxdone <= 1'b0;
   txstart <= 1'b0; rdstart <= 1'b0; romstart <= 1'b0;
   locaddr <= 1'b0;
   md_proc <= 1'b0;
   fp_state <= fp_idle;
	rx_rdy <= 1'b0;
   {bdl_v, bdl_c, bdl_e, bdl_s, bdl_l, bdl_h} <= 6'b0;
	busy <= 1'b0;
	md_mux <= 1'b0;
	st_ena <= 1'b0; st_res <= 1'b0;
end

reg  [1:0]	rxrdy_r, txdone_r;
always @(posedge wb_clk_i) begin
	rxrdy_r[0] <= rxrdyw; rxrdy_r[1] <= rxrdy_r[0];
	txdone_r[0] <= txdonew; txdone_r[1] <= txdone_r[0];
end
wire		rxrdy = rxrdy_r[1];
wire		txdone = txdone_r[1];
 
//************************************************
// Формирователь ответа на цикл шины   
//************************************************
reg reply;
//always @(posedge wb_clk_i, posedge comb_res)
always @(posedge wb_clk_i)
   if (comb_res)			reply <= 1'b0;
   else if (wb_stb_i)	reply <= 1'b1;
   else 						reply <= 1'b0;

assign wb_ack_o = reply & wb_stb_i;    

//**************************************************
// Работа с шиной
//**************************************************
//always @(posedge wb_clk_i, posedge comb_res) begin
always @(posedge wb_clk_i) begin
   if(comb_res) begin
      //******************
      // Сброс модуля
      //******************
      // Снимаем запрос на прерывания
      irq <= 1'b0; interrupt_state <= i_idle; int_req <= 1'b0;
      // Сброс регистра управления
      csr_ri  <= 1'b0; csr_se <= 1'b0; csr_el <= 1'b0;
      csr_il <= 1'b0; csr_xi <= 1'b0; csr_ie <= 1'b0; csr_rl <= 1'b1; csr_xl <= 1'b1;
      csr_bd <= 1'b0; csr_ni <= 1'b0; csr_sr <= 1'b0; csr_re <= 1'b0;
      // Сброс регистра вектора
      if(~res_soft) begin	// Не программный сброс
         var_id <= 1'b0; var_iv <= 8'o0; var_rs  <= 1'b1;
         var_s3 <= 1'b1; var_s2 <= 1'b1; var_s1 <= 1'b1;
         var_ms <= s3; var_os <= s4;
      end
      // Сброс регистра функций
      funcreg <= 3'b0;
      txstart <= 1'b0; rdstart <= 1'b0; romstart <= 1'b0;
      // Сброс регистра сброса
      // Сброс сигнала работы блока MD
      md_proc <= 1'b0; md_cmd <= 1'b0;
      // Сброс регистра готовности режима приема
		rx_rdy <= 1'b0;
		busy <= 1'b0;
		md_mux <= 1'b0;
   end

   // Рабочие состояния
   else begin
      //******************************
      //* Обработка прерывания
      //******************************
      case (interrupt_state)
         // Нет активного прерывания
         i_idle :  begin
            //  Если поднят флаг - переходим в состояние активного прерывания
            if ((csr_ie == 1'b1) & (int_req == 1'b1))  begin
               interrupt_state <= i_req; 
               irq <= 1'b1;                                 // запрос на прерывание
            end 
            // иначе снимаем запрос на прерывание
            else irq <= 1'b0 ;    
         end
         // Формирование запроса на прерывание         
         i_req :   
            if (csr_ie == 1'b0) interrupt_state <= i_idle;  // прерывания запрещены
            else if (iack == 1'b1) begin
				// Если получено подтверждение прерывания от процессора
               irq <= 1'b0;                                 // снимаем запрос
               int_req <= 1'b0;                             // очищаем триггер прерывания
               interrupt_state <= i_wait;                   // переходим к ожиданию окончания обработки
            end 
         // Ожидание окончания обработки прерывания         
         i_wait : if (iack == 1'b0)
            interrupt_state <= i_idle;                   	// ждем снятия сигнала iack
      endcase

      //*********************************************
      //* Обработка шинных транзакций 
      //*********************************************
      // Чтение регистров
      if (bus_read_req == 1'b1) begin
         case (wb_adr_i[3:1])
            3'b000 : begin	// Base + 00
               if(sa_rom_chk)
                  wb_dat_o <= md_mux? {md_status, macval[55:48]} : {8'hFF, macval[55:48]};
               else
                  wb_dat_o <= md_mux? {md_status, macval[7:0]} : {8'hFF, macval[7:0]};
            end
            3'b001 : begin	// Base + 02
               if(sa_rom_chk)
                  wb_dat_o <= md_mux? {errs, macval[63:56]} : {8'hFF, macval[63:56]};
               else
                  wb_dat_o <= md_mux? {errs, macval[15:8]} : {8'hFF, macval[15:8]};
            end
            3'b010 : begin	// Base + 04
               wb_dat_o <= md_mux? {md_out[7:0], macval[23:16]} : {8'hFF, macval[23:16]};
            end
            3'b011 : begin	// Base + 06
               wb_dat_o <= md_mux? {md_out[15:8], macval[31:24]} : {8'hFF, macval[31:24]};
            end
            3'b100 : begin	// Base + 10
               wb_dat_o <= md_mux? {1'b0,md_ctrl, macval[39:32]} : {8'hFF, macval[39:32]};
            end
            3'b101 : begin	// Base + 12
               wb_dat_o <= {8'hFF, macval[47:40]};
            end
            3'b110 : begin	// Base + 14 -- var
               wb_dat_o <= vareg;
            end
            3'b111 : begin	// Base + 16 -- csr
               wb_dat_o <= csr;
            end
         endcase 
      end
      // Запись регистров
      else if (bus_write_req == 1'b1) begin
         if (wb_sel_i[0] == 1'b1) begin   // Запись младшего байта
				case (wb_adr_i[3:1])
					3'b000: begin  // Base	- MD ctrl register
						if(allow_bus_ops & md_mux) begin
							md_reg <= wb_dat_i[4:0];
							md_func <= wb_dat_i[6];
							if(wb_dat_i[5]) begin
								md_cmd <= 1'b1;
								funcreg <= f_mdc;
							end
						end
					end
					3'b001: 			// Base + 02 - MD value
						if(allow_bus_ops) md_val[7:0] <= wb_dat_i[7:0];
					3'b010: 			// Base + 04 - RBDL low
						if(allow_bus_ops) rbdl_lwr[7:1] <= wb_dat_i[7:1];
					3'b011:			// Base + 06 - RBDL high
						if(allow_bus_ops) rbdl_hir[5:0] <= wb_dat_i[5:0];
					3'b100:			// Base + 10 - TBDL low
						if(allow_bus_ops) tbdl_lwr[7:1] <= wb_dat_i[7:1];
					3'b101:			// Base + 12 - TBDL high
						if(allow_bus_ops) tbdl_hir[5:0] <= wb_dat_i[5:0];
					3'b110: begin	// Base + 14 - VAR
						if(allow_bus_ops) begin
							var_id <= wb_dat_i[0];
							var_iv[5:0] <= wb_dat_i[7:2];
						end
					end
					3'b111: begin  // Base + 16 - CSR
						if(allow_bus_ops) begin
							csr_ie <= wb_dat_i[6];
							if(wb_dat_i[7] == 1'b1) begin
								csr_xi <= 1'b0;
								csr_ni <= 1'b0;
							end
							csr_re <= wb_dat_i[0];
							// Только для PDP-11. Для алгоритма смотри доку
							if((csr_bd == 1'b1) & (wb_dat_i[3] == 1'b0)) begin
								csr_bd <= wb_dat_i[3]; funcreg <= f_bdr;
							end
							else
								csr_bd <= wb_dat_i[3];
						end
						csr_sr <= wb_dat_i[1];  // 1 - 0 => программный сброс
					end
				endcase
         end
         if(wb_sel_i[1] == 1'b1) begin    // Запись старшего байта
            if(allow_bus_ops) begin
               case (wb_adr_i[3:1])
                  3'b000: begin  // Base	- MD ctrl register
							md_mux <= wb_dat_i[15];
                  end
                  3'b001:        // Base + 02 - MD value
							md_val[15:8] <= wb_dat_i[15:8];
                  3'b010: begin  // Base + 04 - RBDL low
                     rbdl_lwr[15:8] <= wb_dat_i[15:8];
                  end
                  3'b011: begin  // Base + 06 - RBDL high
                     csr_rl <= 1'b0;
                  end
                  3'b100: begin  // Base + 10 - TBDL low
							tbdl_lwr[15:8] <= wb_dat_i[15:8];
                  end
                  3'b101: begin  // Base + 12 - TBDL high
							csr_xl <= 1'b0;
                  end
                  3'b110: begin  // Base + 14 - VAR
                     var_iv[7:6] <= wb_dat_i[9:8];
                     var_rs <= wb_dat_i[13];
                     var_ms <= wb_dat_i[15];
                  end
                  3'b111: begin  // Base + 16 - CSR
                     csr_il <= wb_dat_i[8];
                     csr_el <= wb_dat_i[9];
                     csr_se <= wb_dat_i[10];
                     if(wb_dat_i[15] == 1'b1) csr_ri <= 1'b0;
                  end
               endcase
            end
         end
      end
      else begin  // Обработка выбора функций
         if(var_rs) begin
            funcreg <= f_st;
         end
      end
      //*********************************************
      //* Выполнение функций
      //*********************************************
		case(funcreg)
			f_idl: begin
				busy <= 1'b0;
			end
			f_mdc: begin
				busy <= 1'b1;
				case({md_status[7],md_cmd,md_proc})
					3'b100: funcreg <= f_idl;
					3'b010: md_proc <= 1'b1;
					3'b011: md_cmd <= 1'b0;
					3'b101: begin
						md_proc <= 1'b0;
					end
				endcase
			end
			f_st: begin    // Самотестирование модуля (нужна задерка в 5 сек.)
				busy <= 1'b1;
				var_s3 <= 1'b0;                              // Результат тестирования
				var_s2 <= 1'b0;                              // безошибочный по всем модулям
				var_s1 <= 1'b0;                              // ROM CRC, RAM, 68000, QIC, QNA, SA ROM. LANCE 
				var_rs <= 1'b0;                              // Сброс бита самотестирования
				funcreg <= f_idl;                            // сброс кода функции
			end
			f_bdr: begin   // Выгрузка 4K ROM в память хоста
				if(intextmode == 1'b1) begin
					if((rdproc == 1'b0) & (romstart == 1'b0)) begin
						busy = 1'b1;
						romstart <= 1'b1;                      // запуск режима приема
					end
					else if((romstart == 1'b1) & (rdproc == 1'b1)) begin
						romstart <= 1'b0;
						csr_ri <= 1'b1;
						funcreg <= f_idl;								// сброс кода функции
					end
				end
				else begin
					csr_ni <= 1'b1;                           // флаг ощибки
					funcreg <= f_idl;									// сброс кода функции
				end
			end
			default: funcreg <= f_idl;
		endcase
//
// Инкрементация BDL регистров (младшая часть)
		if(tbdli) begin
			tbdl_lwr <= tbdl_lwr + 3'b110;						// смещение к следующему BDL блоку (+ 6 слов)
		end
		if(rbdli) begin
			rbdl_lwr <= rbdl_lwr + 3'b110;						// смещение к следующему BDL блоку (+ 6 слов)
		end
//
// Передача сетевого пакета
		if(~csr_xl) begin												// Если готовы BDL-регистры
			if(txproc == 1'b0 & txstart == 1'b0) begin
				busy <= 1'b1;
				txstart <= 1'b1;										// запуск режима передачи
         end
         else if(txstart == 1'b1 & txproc == 1'b1) begin
				txstart <= 1'b0;
            funcreg <= f_idl;										// сброс кода функции
            csr_xl <= 1'b1;										// флаг завершения работы с XBDL
				busy <= 1'b0;
            if(txerr | rderr) begin
					if(nxm) csr_ni <= 1'b1;							// флаг ощибки
            end
            else begin
					csr_xi <= 1'b1;									// флаг передачи пакета
               int_req <= 1'b1;									// флаг требования прерывания
            end
         end
      end
//
// Прием сетевого пакета
		if(rxrdy & ~csr_rl) begin									// Если готовы BDL-регистры и есть сигнал готовности пакета ...
			busy <= 1'b1;
         rx_rdy <= 1'b1;                                 // ... установить флаг готовности принять пакет
      end
      if(rx_rdy) begin                                   // Если готовы принять пакет и
         if(rdproc == 1'b0 & rdstart == 1'b0) begin      // если операция чтения не активна ...
            rdstart <= 1'b1;                             // ... - запуск режима приема.
         end
         else if(rdstart == 1'b1 & rdproc == 1'b1) begin
            rdstart <= 1'b0;
            csr_rl <= 1'b1;                              // флаг завершения работы с RBDL
				rx_rdy <= 1'b0;										// Сброс регистров готовности режима приема
				busy <= 1'b0;
            if(rderr | txerr) begin                      // Если возникли ошибки -
               if(nxm) csr_ni <= 1'b1;                   // установить флаг ощибки,
            end
            else begin                                   // Нет ошибок -
               csr_ri <= 1'b1;                           // установить флаг приема пакета,
               int_req <= 1'b1;                          // установить флаг требования прерывания.
            end
         end
      end
   end
end

//************************************************
// Прием/передача пакетов
//************************************************
//always @(posedge wb_clk_i, posedge comb_res) begin
always @(posedge wb_clk_i) begin
   if (comb_res) begin
   // Сброс
      fp_state <= fp_idle;
      txproc <= 1'b0; txerr <= 1'b0;   // флаги режима передачи
      rdproc <= 1'b0; rderr <= 1'b0;   // флаги режима приема
      txrdy <= 1'b0; rxdone <= 1'b0;   // флаги готовности
      dmawr <= 1'b0; dmard <= 1'b0;	   // флаги режима DMA
      baddr <= 11'o0;                  // регистр адреса
      txcntb <= 11'o0;                 // счетчик байтов передачи
		tbdli <= 1'b0; rbdli <= 1'b0;		// флаги инремент. BDL регистров
		bdlbus <= 1'b0;						// коммутация шины BDL на вход DMA
		setupreg <= 1'b0;						// S-бит
		mcasf <= 1'b0;
		promf <= 1'b0;
		sanity <= 3'b101;
		st_ena <= 1'b0; st_res <= 1'b0;
   end
   // Рабочие состояния
   else  begin
      case(fp_state)
         fp_idle: begin	// ожидание
            stwwe <= 1'b0; locaddr <= 1'b0;
            case(fp_mode)
               4'b0001: fp_state <= fp_txp0;
               4'b0010: fp_state <= fp_rdp0;
               4'b0100:	fp_state <= fp_rom1;
               default:	fp_state <= fp_idle;
            endcase
         end
//=====================================================================================================================
// Работа с BDL, приемная часть (общая как для приема, так и для передачи пакета даных)
         fp_bdl0: begin // Передача 1 слова bdl регистра по каналу DMA
            if(dmacomplete == 1'b0 & dmard == 1'b0) begin
               dmard <= 1'b1;                            // устанавливаем флаг чтения по каналу DMA
               haddr_sw[15:1] <= haddr[15:1];            // физический адрес (копия)
               rderr <= 1'b0; txerr <= 1'b0;             // сброс кода ошибки
					bdlbus <= 1'b1;
            end
            else if(dmard == 1'b1 & dmacomplete == 1'b1) begin
               dmard <= 1'b0;                            // снимаем флаг чтения по каналу DMA
					bdlbus <= 1'b0;
               if (nxm == 1'b0) begin                    // запись окончилась без ошибок
                  baddr <= 11'd01;                       // пропустить резервное слово BDL
                  wcount <= 12'o7775;                    // число слов BDL (-3)
                  haddr[21:1] <= haddr[21:1] + 1'b1;     // смещение на 1 слово
                  fp_state <= fp_bdl1;                   // переход к приему 3-х слов данных
               end
               else begin
                  fp_state <= fp_bdle;							// таймаут - завершение
               end
            end
         end
         fp_bdl1: begin // Прием 3-х слов bdl регистра по каналу DMA
            if(dmacomplete == 1'b0 & dmawr == 1'b0) begin
               dmawr <= 1'b1;                            // устанавливаем флаг записи по каналу DMA
            end
            else if(dmawr == 1'b1 & dmacomplete == 1'b1) begin
               dmawr <= 1'b0;                            // снимаем флаг записи по каналу DMA
               if (nxm == 1'b0) begin                    // запись окончилась без ошибок
                  baddr <= 11'd01;                       // пропустить резервное слово BDL
                  locaddr <= 1'b1;                       // местный адрес
                  fp_state <= fp_bdl2;                   // переход к извлечению данных
               end
               else begin
                  fp_state <= fp_bdle;                   // таймаут - завершение
               end
            end
         end
         fp_bdl2: begin // Биты описания и старшие биты адреса
            {bdl_v, bdl_c, bdl_e, bdl_s, bdl_rez, bdl_l, bdl_h} <= bdldat[15:6];  // биты описания
            haddr[21:16] <= bdldat[5:0];						// старшая часть физического адреса
            baddr <= baddr + 1'b1;                       // следующее слово
            fp_state <= fp_bdl3;                         // переход к младшей части адреса
         end
         fp_bdl3: begin // Младшая часть адреса
            haddr[15:1] <= bdldat[15:1];						// младшая часть физического адреса
            baddr <= baddr + 1'b1;                       // следующее слово
            fp_state <= fp_bdl4;                         // переход к счетчику слов
         end
         fp_bdl4: begin	// Счетчик байтов
				bcount <= bdl_s? {~bdldat[11:0] + 1'b1, bdl_l} : {~bdldat[11:0] + 1'b1, 1'b0};	
				fp_state <= fp_bdl5;
			end
			fp_bdl5: begin
				if(bdl_s) begin										// Если Setup пакет:
					if(bdl_l) bcount <= bcount -2'b10;			// - установден L бит - коректировка кол-ва байтов
					wcount <= ~swcount + 1'b1;						// - новое значение числа слов передачи
					fp_state <= fp_bdls;								// - переход на обработку контрольного регистра
				end
				else begin												// Не Setup пакет:
					bcount <= bcount - bdl_l - bdl_h;			// -  коректировка кол-ва байтов
					wcount <= bdldat[11:0];							// - значение числа слов передачи
					st_res <= 1'b1;									// - сброс sanity timer
					fp_state <= fp_bdlc;								// - пропуск обработки контрольного регистра
				end
			end
			fp_bdls: begin	// Обработка контрольного регистра Setup пакета
				mcasf <= bcount[0];
				promf <= bcount[1];
				sanity <= bcount[6:4];
				st_ena <= csr_se;
				if(bcount < 12'o0200)
					fp_state <= fp_bdle;
				fp_state <= fp_bdlc;
			end
			fp_bdlc: begin	// Проверка валидности пакета данных
            if(bdl_v) begin										// установлен бит V?
					fp_state <= fp_next;                      // да - переход к передаче
					baddr <= 11'o0;									// сброс адреса
					locaddr <= 1'b0;									// адрес сформированный в модуле DMA
				end
            else
               fp_state <= fp_bdle;								// нет - завершение.
         end
         fp_bdle: begin	// Переход на завершение
            case(fp_next)
               fp_txp1: begin
                  fp_state <= fp_ftxp;                   // уход на завершение цикла передачи пакета данных
                  txerr <= 1'b1;                         // устанавливаем флаг ошибки
               end
               fp_rlng: begin
                  fp_state <= fp_frdp;                   // уход на завершение цикла приема пакета данных
                  rderr <= 1'b1;                         // устанавливаем флаг ошибки
                  rxdone <= 1'b1;                        // устанавливаем флаг завершения чтения
               end
            endcase
         end
//=====================================================================================================================
// Цикл передачи пакета данных
         fp_txp0: begin	// Начальная подготовка
            haddr[21:1] <= {tbdl_hir[5:0], tbdl_lwr[15:1]}; // физический адрес
            baddr <= 11'o0;										// начальный адрес
            wcount <= 12'o7777;									// число слов BDL (-1)
				fp_next <= fp_txp1;									// точка входа после завершения BDL цикла
            fp_state <= fp_bdl0;									// переход к  BDL циклу
         end
         fp_txp1: begin // Передача данных сетевого пакета по каналу DMA
				setupreg <= bdl_s;
            if(bdl_e) begin                              // Установлен бит E?
               if(dmacomplete == 1'b0 & dmawr == 1'b0) begin
                  dmawr <= 1'b1;                         // устанавливаем флаг записи по каналу DMA;
                  txcntb <= {wcount[9:0],1'b0} - bdl_h - bdl_l;   // число байтов передачи;
               end
               else if(dmawr == 1'b1 & dmacomplete == 1'b1) begin
                  dmawr <= 1'b0;                         // Снимаем флаг записи по каналу DMA
                  if (nxm == 1'b0) begin                 // Запись окончилась без ошибок?
                     txrdy <= 1'b1;                      // Да: установить бит готовности работы модуля Ethernet,
                     fp_state <= fp_etxp;                // переход к передаче данных.
                  end
                  else begin
                     txerr <= 1'b1;                      // Нет: установить флаг ошибки,
                     fp_state <= fp_ftxp;                // переход на завершение.
                  end
               end
            end
            else begin                                   // Биты V & E не установлены:
               txerr <= 1'b1;                            // установить флаг ошибки,
               fp_state <= fp_ftxp;                      // переход на завершение.
            end
         end
         fp_etxp: begin // Ожидание завершение передачи Ethernet кадра
            if(txrdy & txdone) begin
               txrdy <= 1'b0;                            // сбросить бит готовности работы модуля Ethernet
               fp_next <= fp_ftxp;                       // точка входа после передачи статусной информации
               fp_state <= fp_stw0;                      // формирование и передача статусной информации
            end
         end
         fp_ftxp: begin // Завершение цикла передачи пакета данных
            if(txstart == 1'b0) begin
               txproc <= 1'b0;
					if(txerr)	fp_state <= fp_idle;
               else begin
						fp_state <= fp_tbdl;
						tbdli <= 1'b1;
					end
            end
            else txproc <= 1'b1;
         end
			fp_tbdl: begin // Следующий BDL блок
				if(~tbdli) begin
					fp_state <= fp_txp0;								// цикл
				end
				else begin
					tbdli <= 1'b0;
				end
			end
//=====================================================================================================================
// Цикл приема пакета данных
         fp_rdp0: begin // Начальная подготовка
            haddr[21:1] <= {rbdl_hir[5:0], rbdl_lwr[15:1]}; // физический адрес
            baddr <= 11'o0;                              // начальный адрес
            wcount <= 12'o7777;                          // число слов BDL (-1)
            fp_next <= fp_rlng;                          // точка входа после завершения BDL цикла
            fp_state <= fp_bdl0;                         // переход к  BDL циклу
         end
         fp_rlng: begin
            wcount[9:0] <= (((~rxcntb[10:0]) + 1'b1) >> 1); // вычисление числа слов передачи
            fp_state <= fp_rdp1;                         // переход к DMA-циклу
         end
         fp_rdp1: begin // чтение сетевого пакета
            if (dmacomplete == 1'b0 & dmard == 1'b0) begin
               baddr <= 11'o0;                           // сброс адреса
               wcount <= {2'b11, wcount[9:0]};           // формирование числа слов передачи
               dmard <= 1'b1;                            // устанавливаем флаг чтения по каналу DMA
            end
            else if(dmard == 1'b1 & dmacomplete == 1'b1) begin
               dmard <= 1'b0;                            // снимаем флаг чтения по каналу DMA
               if (nxm == 1'b0) begin                    // чтение окончилось без ошибок
                  rxdone <= 1'b1;                        // устанавливаем флаг завершения чтения
                  fp_state <= fp_erdp;                   // переход к завершающему шагу
               end
               else begin
                  rderr <= 1'b1;                         // устанавливаем флаг ошибки
                  fp_state <= fp_frdp;                   // переход на этап завершения
               end
            end
         end
         fp_erdp: begin // Формирование длины пакета и переход на передачц статусных слов
            if(loop)													// Setup или loop пакет
					st_rbl <= rxcntb;									// - иначе второй вариант RBL.
            else														// Нормальный режим ...
               st_rbl <= rxcntb + 11'd1988;					// ... -60 байт 
            fp_next <= fp_frdp;                          // точка входа после передачи статусной информации
            fp_state <= fp_stw0;                         // формирование и передача статусной информации
         end
         fp_frdp: begin // Завершение цикла приема пакета данных
            if(~rxrdy & rxdone) rxdone <= 1'b0;          // Сброс сигнала готовности приема данных
            if(rdstart == 1'b0) begin
               rdproc <= 1'b0;
					if(rderr)	fp_state <= fp_idle;
               else begin
						fp_state <= fp_rbdl;
						rbdli <= 1'b1;
					end
            end
            else rdproc <= 1'b1;
         end
			fp_rbdl: begin // Следующий BDL блок
				if(~rbdli) begin
					fp_state <= fp_rdp0;								// переход к  BDL циклу
				end
				else begin
					rbdli <= 1'b0;
				end
			end
//=====================================================================================================================
// Цикл передачи блока ROM
         fp_rom1: begin // чтение сетевого пакета
            if (dmacomplete == 1'b0 & dmawr == 1'b0) begin
               haddr[21:1] <= {rbdl_hir[5:0], rbdl_lwr[15:1]}; // физический адрес
               baddr <= 11'o0;                           // сброс адреса
               wcount <= 12'o4000;                       // число слов BDL (-2048)
               rderr <= 1'b0;
               dmard <= 1'b1;                            // устанавливаем флаг чтения по каналу DMA
            end
            else if(dmard == 1'b1 & dmacomplete == 1'b1) begin
               dmard <= 1'b0;                            // снимаем флаг чтения по каналу DMA
               fp_state <= fp_rom2;                      // переход к завершающему шагу
               if(nxm)                                   // операция окончилась по таймауту?
                  rderr <= 1'b1;                         // устанавливаем флаг ошибки
            end
         end
         fp_rom2: begin
            if(romstart == 1'b0) begin
               rdproc <= 1'b0;
               fp_state <= fp_idle;
            end
            else rdproc <= 1'b1;
         end
//=====================================================================================================================
// Формирование и передача статусной информации
         fp_stw0:	begin
            fp_state <= fp_stw1;
            baddr <= 11'o4;                              // начальный адрес
            locaddr <= 1'b1;                             // местный адрес
			end
         fp_stw1:	begin
            fp_state <= fp_stwa;
            stwwe <= 1'b1;
            if(fp_next == fp_ftxp) begin                 // если режим передачи
               stwdat <= {1'b0,st_txerr,14'o0};				// статусное слово операции передачи
            end
            else begin                                   // иначе статусное слово операции приема
					if(setupreg) begin
						stwdat <= {8'b00100111,st_rbl[7:0]};
						setupreg <= 1'b0;
					end
					else
						stwdat <= {1'b0,st_rxerr,st_esetup,1'b0,st_runt,st_rbl[10:8],5'b11111,st_frame,st_crcerr,1'b0};
            end
         end
         fp_stwa:	begin
            baddr <= baddr + 1'b1;
            stwwe <= 1'b0;
            fp_state <= fp_stw2;
         end
         fp_stw2:	begin
            fp_state <= fp_stwh;
            stwwe <= 1'b1;
            if(fp_next == fp_ftxp) begin                 // если режим передачи
               stwdat <= {16'o0};								// статусное слово операции передачи
            end
            else begin                                   // иначе
               stwdat <= {st_rbl[7:0],st_rbl[7:0]};		// статусное слово операции приема
            end
         end
         fp_stwh: begin
            stwwe <= 1'b0;
            if (dmacomplete == 1'b0 & dmawr == 1'b0) begin
               haddr[15:1] <= haddr_sw[15:1] + 3'o4;     // смещение адреса
               baddr <= 11'o4;
               locaddr <= 1'b0;                          // адрес сформированный в модуле DMA
               wcount <= 12'o7776;                       // число слов передачи (-2)
               dmard <= 1'b1;                            // устанавливаем флаг чтения по каналу DMA
					bdlbus <= 1'b1;
            end
            else if(dmard == 1'b1 & dmacomplete == 1'b1) begin
               dmard <= 1'b0;                            // снимаем флаг чтения по каналу DMA
					bdlbus <= 1'b0;
					st_res <= 1'b0;									// убираем сигнал сброса sanity timer
               fp_state <= fp_next;                      // переход к завершающему шагу
               if(nxm)                                   // операция окончилась по таймауту?
                  rderr <= 1'b1;                         // устанавливаем флаг ошибки
            end
         end
         default: fp_state <= fp_idle;
      endcase
   end
end


//************************************************
// DMA
//************************************************
//wire [15:0] rxdbus = romstart? romdat : ((fp_state == fp_stwh)? stwout : {mrxdbus[7:0],mrxdbus[15:8]});
wire [15:0] rxdbus = romstart? romdat : (bdlbus? bdldat : {mrxdbus[7:0],mrxdbus[15:8]});

dma dmamod(
   .clk_i(wb_clk_i),          // тактовая частота шины
   .rst_i(comb_res),          // сброс
   .dma_req(dma_req),         // запрос DMA
   .dma_gnt(dma_gnt),         // подтверждение DMA
   .dma_adr_o(dma_adr_o),     // выходной адрес при DMA-обмене
   .dma_dat_i(dma_dat_i),     // входная шина данных DMA
   .dma_dat_o(dma_dat_o),     // выходная шина данных DMA
   .dma_stb_o(dma_stb_o),     // строб цикла шины DMA
   .dma_we_o(dma_we_o),       // направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
   .dma_ack_i(dma_ack_i),     // Ответ от устройства, с которым идет DMA-обмен
   .wstart(dmawr),            // Функция записи
   .rstart(dmard),            // Функция чтения
   .iocomplete(dmacomplete),  // Флаг завершения операции
   .rxdbus(rxdbus),           // Шина данных чтения
   .txdbus(mtxdbus),          // Шина данных записи
   .baddr_i(baddr),           // Адрес буфера
   .baddr_o(baddrinc),        // Адрес буфера + 1
   .haddr(haddr),             // Физический адрес памяти
   .wcount(wcount),           // Счетчик слов
   .nxm(nxm)                  // Флаг ошибки
);

//************************************************
// Генерация MD clock
//************************************************
wire md_clock;
wire md_evt;
mdc_clk mclk(
	.clock(wb_clk_i),
	.rst(comb_res),
	.mdcclk(md_clock),
	.mdsevt(md_evt)
);
assign e_mdc = md_clock;

wire sanres = st_res | comb_res;		// Комбинированный сброс таймера
santim santm(
	.clock(md_clock),
	.rst(sanres),
	.pwse(pwse),
	.sanity(sanity),
	.ena(st_ena),
	.out(nbdcok)
);

endmodule
