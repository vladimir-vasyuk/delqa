//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//=================================================================================
module ether(
   input          rst,        // Сброс
	input				mcast,		// Сигнал mulsicast
	input				promis,		// Сигнал promiscuous
   input          txrdy,      // Сигнал готовности данных передачи
   input          rxdone,     // Сигнал подтверждения приема
   input  [10:0]  txcntb,     // Счетчик данных передачи (байт)
   input  [31:0]  etxdbus,    // Шина данных блока передачи (память -> ether модуль)
   input  [4:0]   lbmode,     // Режим работы модуля Ethernet
   output [8:0]   etxaddr,    // Регистр адреса блока передачи (память -> ether модуль)
   output [8:0]   erxaddr,    // Регистр адреса блока приема (ether модуль -> память)
   output [31:0]  erxdbus,    // Шина данных блока приема (ether модуль -> память)
   output [10:0]  rxcntb,     // Счетчик данных приема (байт)
   output         erxwrn,     // Сигнал записи
   output         rxclkb,     // Синхросигнал канала приема (запись в буферную память)
   output         rxrdy,      // Данные приема готовы
   output         txdone,     // Передача завершена
   output [7:0]   errs,	      // Ошибки приема/передачи
//	input  [47:0]	macbus,     // MAC адрес
//
	input          e_rxc,      // Синхросигнал канала приема
	input          e_rxdv,     // Сигнал готовности данных канала приема
	input          e_rxer,     // Ошибка канала приема
	input  [7:0]   e_rxd,      // Данные канала приема
	input          e_crs,      // Сигнал наличия несущей
	input          e_txc,      // Синхросигнал канала передачи
	output         e_txen,     // Сигнал разрешения передачи
	output         e_txer,     // Сигнал ошибки канала передачи
	output [7:0]   e_txd,      // Данные канала передачи
	output         e_rst,      // Сброс, активный - низкий
	output         e_gtxc,     // Опорный синхросигнал для 1Gb
	inout          e_mdio,     // Блок управления - линия данных
	input          md_clk,     // Блок управления - синхросигнал
	input          md_evt,     // Блок управления - сигнал опроса состояния
	input  [6:0]   md_ctrl,    // Блок управления - сигналы управления от компьютера
	input  [15:0]  md_val,     // Блок управления - данные записи
	output [15:0]  md_out,     // Блок управления - данные чтения
	output [7:0]   md_status   // Блок управления - данные состояния
);

assign e_rst = ~rst;
assign e_gtxc = e_rxc;
//
//======================= CRC=======================//
wire [31:0] crctx;      // CRC канала передачи
wire [31:0] crcrx;      // CRC канала приема 
wire        crcretx;    // Сигнал сброса CRC канала передачи
wire        crcentx;    // Сигнал разрешения CRC канала передачи
wire        crcrerx;    // Сигнал сброса CRC канала приема
wire        crcenrx;    // Сигнал разрешения CRC канала приема

//======================= MDC ======================//
wire        mdc_err;    // Сигнал ошибки

//================== Прием/передача =================//
//reg         skipb;      // Пропуск байта (DescriptorBits[6])
wire			skipb;		// Пропуск байта (DescriptorBits[6])
wire        rxena;      // Разрешение приема
wire        rx_crc_err; // CRC ошибка канала приема
wire        rx_err;     // Ошибка канала приема
wire        crs_err;    // Отсутствие несущей
assign crs_err = (e_rxer & (~e_rxdv))? 1'b1 : 1'b0;
assign errs = {3'd0, crs_err, mdc_err, e_txer, rx_err, rx_crc_err};

//================= Внутренняя петля ================//
//reg         loop;
//reg         ext_loop;
//reg         rx_enable;
//reg         setup;
wire			loop;
wire			ext_loop;
wire			rx_enable;
wire			setup;

// ===== Gigabit mode - speed=1000 and link=OK ======//
wire        gbmode;
assign gbmode	= ((md_status[6:5] == 2'b10) & (md_status[0] == 1'b1))? 1'b1 : 1'b0;

//===== Мультиплексор 4->8 входной шины данных ======//
wire        rxdv;          // Сигнал готовности данных блока приема
wire        rxer;          // Сигнал ошибки данных канала приема
wire        rxerm;         // Выходной сигнал ошибки данных канала приема
wire [7:0]  rxdb;          // Шина данных канала приема
wire [7:0]  ddinm;         // Мультиплексированные данные для 10Mb-100Mb
wire        rxclkm;        // Новый синхросигнал для 10Mb-100Mb
wire        rxdvm;         // Выходной сигнал готовности данных блока приема
wire        rxclk;         // Синхросигнал канала приема
ddin dd_in(
   .rxclk_i(e_rxc),
   .rst(rst),
   .rxdv(e_rxdv),
   .dat_i(e_rxd[3:0]),
   .dat_o(ddinm),
   .rxclk_o(rxclkm),
   .rxdv_o(rxdvm),
   .rxer_o(rxerm)
);
assign rxclk = gbmode? e_rxc : rxclkm;
assign rxdb = gbmode? e_rxd : ddinm;                                       // Мультиплексированные данные
assign {rxdv, rxer} = gbmode? {e_rxdv, e_rxer} : {rxdvm, rxerm | e_rxer};  // Сигналы управления

//===== Демультиплексор 8->4 выходной шины данны =====//
wire [3:0]  ddoutm;        // Выходные данные для 10Mb-100Mb
wire        txens, txers;  // Управляющие сигналы канала передачи
wire        txeno, txero;  // Управляющие сигналы канала передачи
wire [7:0]  txdb;          // Входные данные канала передачи
wire        txclk;         // Синхросигнал канала передачи
wire        txclkm;        // Новый синхросигнал для 10Mb-100Mb

ddout dd_out(
   .txclk_i(e_txc),
   .rst(rst),
   .txen_i(txens),
   .dat_i(txdb),
   .dat_o(ddoutm),
   .txclk_o(txclkm),
   .txen_o(txeno),
   .txerr_o(txero)
);
assign txclk = gbmode? e_gtxc : txclkm;                                    // Синхросигнал канала передачи
assign e_txd[7:0] = gbmode? txdb[7:0] : {4'o0,ddoutm[3:0]};                // Демультиплексированные данные
assign {e_txen, e_txer} = loop? {1'b0, 1'b0} : (gbmode? {txens, txers} : {txeno, txero | txers});  // Сигналы управления

//======== Обработка данных канала передачи ========//
wire        txclkl;        // Синхросигнал канала передачи с учетом петли
assign txclkl	= loop? e_rxc : txclk;

ethsend ethsendm(
   .clk(txclkl),
   .clr(rst),
   .txena(txrdyl),
   .txdone(txdone),
   .txen(txens),
   .dataout(txdb),
   .crc(crctx),
   .txbdata(etxdbus),
   .txbaddr(etxaddr),
   .txcntb(txcntb),
   .skipb(skipb),
   .crcen(crcentx),
   .crcre(crcretx),
   .err_gen(txers)
);

//========= Обработка данных канала приема =========//
wire [7:0]  rxdbl;         // Шина данных канала приема с учетом петли
wire        rxclkl;        // Синхросигнал канала приема с учетом петли
wire        rxdvl;         // Сигнал готовности данных блока приема с учетом петли
wire        rxerl;         // Сигнал ошибки данных канала приема с учетом петли

assign rxdbl = loop? txdb : rxdb;
assign rxdvl = loop? txens : rxdv;
assign rxclkl = loop? ~e_rxc : rxclk;
assign rxerl = loop? txers : rxer;
assign rxclkb = rxclkl;
assign rxena = loop? loop : rx_enable;

ethreceive ethrcvm(
   .clk(rxclkl),
   .clr(rst),
   .rxena(rxena),
   .datain(rxdbl),
   .rxdv(rxdvl),
   .rxer(rxerl),
   .rxcntb(rxcntb),
   .rxbaddr(erxaddr),
   .rxbdata(erxdbus),
   .rxwrn(erxwrn),
   .rxrdy(rxrdy),
// .mymac(macbus),
   .rxdone(rxdonel),
   .crc(crcrx),
   .crcen(crcenrx),
   .crcre(crcrerx),
   .err_gen(rx_err),
   .err_crc(rx_crc_err)
);

//==== Контрольная сумма данных канала передачи ====//
crc_n crc_tx(
   .clk(txclkl),
   .rst(crcretx),
   .data_in(txdb),
   .crc_en(crcentx),
   .crc_out(crctx)
);
//crc crc_tx(
//	.clk(txclkl),
//	.reset(crcretx),
//	.data_in(txdb),
//	.enable(crcentx),
//	.crc(crctx)
//);

//===== Контрольная сумма данных канала приема =====//
crc_n crc_rx(
   .clk(rxclkl),
   .rst(crcrerx),
   .data_in(rxdbl),
   .crc_en(crcenrx),
   .crc_out(crcrx)
);
//crc crc_rx(
//	.clk(rxclkl),
//	.reset(crcrerx),
//	.data_in(rxdbl),
//	.enable(crcenrx),
//	.crc(crcrx)
//);

//================ Блок управления =================//
mdc mdcm(
   .clock(md_clk),
   .rst(rst),
   .evt(md_evt),
   .mdiol(e_mdio),
   .err(mdc_err),
   .ctrl(md_ctrl),
   .val(md_val),
   .out(md_out),
   .status(md_status)
);

reg  [1:0]	rx_ena_r, loop_r, eloop_r, skipb_r, setup_r;
reg  [1:0]	txrdy_r, rxdn_r;
wire			txrdyl, rxdonel;

always @(posedge e_rxc) begin
	rx_ena_r[0] <= lbmode[0]; rx_ena_r[1] <= rx_ena_r[0];
	loop_r[0] <= lbmode[1]; loop_r[1] <= loop_r[0];
	eloop_r[0] <= lbmode[2]; eloop_r[1] <= eloop_r[0];
	skipb_r[0] <= lbmode[3]; skipb_r[0] <= skipb_r[1];
	setup_r[0] <= lbmode[4]; setup_r[0] <= setup_r[1];
	txrdy_r[0] <= txrdy; txrdy_r[1] <= txrdy_r[0];
	rxdn_r[0] <= rxdone; rxdn_r[1] <= rxdn_r[0];
end

assign rx_enable = rx_ena_r[1];
assign loop = loop_r[1];
assign ext_loop = eloop_r[1];
assign skipb = skipb_r[1];
assign setup = setup_r[1];
assign txrdyl = txrdy_r[1];
assign rxdonel = rxdn_r[1];

endmodule
