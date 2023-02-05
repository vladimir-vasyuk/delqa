//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль передачи кадра данных
//=================================================================================
module ethsend(
   input             clk,        // Синхросигнал
   input             clr,        // Сигнал сброса
   input             txena,      // Сигнал готовности данных передачи
   output reg        txdone,     // Сигнал завершения передачи
   output reg        txen,       // Сигнал достоверности данных
   output reg [7:0]  dataout,    // Шина выходных данных
   input      [31:0] crc,        // Шина CRC
   input      [31:0] txbdata,    // Шина входных данных
   output reg [8:0]  txbaddr,    // Регистр адреса буферной памяти
   input      [10:0] txcntb,     // Счетчик переданных байтов
   input             skipb,      // Пропуск байта (H-bit of the "Address Descriptor Bits")
   output reg        crcen,      // Cигнал разрешения вычисления CRC
   output reg        crcre,      // Cигнал сброса CRC
   output            err_gen     // Сигнал общей ошибки (не задействован)
);

reg         txer;
assign err_gen = txer;


reg  [10:0] i;          // Внутренний счетчик
reg  [31:0] bufdat;     // Буфер данных передачи
reg  [1:0]  bc;         // Счетчик байтов

// Конечный автомат канала передачи
localparam IDLE		= 3'd0;
localparam SENDPRE	= 3'd1;
localparam SENDDATA	= 3'd2;
localparam SENDCRC	= 3'd3;
localparam TXDELAY	= 3'd4;
localparam WAITDONE	= 3'd5;
reg  [2:0]  tx_state;

// Инициализация
initial
   begin
      tx_state <= IDLE;
   end

// Основной блок
always@(negedge clk, posedge clr) begin
//always@(negedge clk) begin
   if(clr) tx_state <= IDLE;
   else begin
      case(tx_state)
         IDLE: begin
            txer <= 1'b0;              // Сброс сигнала ошибки
            txen <= 1'b0;              // Сброс сигнала достоверности данных
            crcen <= 1'b0;             // Сброс сигнала разрешения вычисления CRC
            crcre <= 1'b1;             // Установка сигнала сброса CRC
            txbaddr <= 9'o0;           // Начальное значение регистра адреса
            i <= 11'b0;                // Сброс счетчика
            bc <= 2'o0;                // Сброс счетчика байтов
            txdone <= 1'b0;            // Сброс сигнала завершения передачи
            if(txena == 1'b1) begin
               tx_state <= SENDPRE;
            end
         end
         SENDPRE: begin    // Сигналы сихронизации и начала кадра
            txen<=1'b1;                // Установить сигнал достоверности данных
            crcre<=1'b1;               // Сброс crc
            if(i < 7) begin            // 7 байтов ...
               dataout[7:0] <= 8'h55;  // ... преамбулы
               i <= i + 1'b1;
            end
            else begin
               dataout[7:0] <= 8'hD5;  // Байт разделитель
               if(skipb)               // Если установлен H-bit, ...
                  bc <= 2'o1;          // ... пропкустить один байт.
               i <= txcntb;            // Число байтов для передачи
               bufdat <= txbdata;      // Данные для передачи в буфер
               tx_state <= SENDDATA;   // Переход к передачи данных
            end
         end
         SENDDATA: begin   // Передача данных
            crcen <= 1'b1;             // Сигнал разрешения вычисления CRC
            crcre <= 1'b0;             // Убрать сигнал сброса CRC
            if(i == 11'h7FF) begin     // Последний байт данных?
               i <= 11'h0;             // Да - обнулить счетчик ...
               tx_state <= SENDCRC;    // ... и на завершение
               // Передача последнего байта данных
               if(bc == 2'o0) begin
                  dataout[7:0] <= bufdat[7:0];
               end
               else if(bc == 2'o1) begin
                  dataout[7:0] <= bufdat[15:8];
                  bc <= 2'o0;
               end
               else if(bc == 2'o2) begin
                  dataout[7:0] <= bufdat[23:16];
                  bc <= 2'o0;
               end
               else if(bc == 2'o3) begin
                  dataout[7:0] <= bufdat[31:24];
                  bc <= 2'o0;
               end
            end
            else begin                 //  Нет ...
               i <= i + 1'b1;          // ... инкремент счетчика ...
               // Передача очередного байта данных
               if(bc == 2'o0) begin
                  dataout[7:0] <= bufdat[7:0];  // ... данные на шину передачи, ...
                  bc <= bc + 1'b1;              // ... инкремент счетчика байтов, ...
                  txbaddr <= txbaddr + 1'b1;    // ... инкремент регистра адреса.
               end
               else if(bc == 2'o1) begin
                  dataout[7:0] <= bufdat[15:8];
                  bc <= bc + 1'b1;
               end
               else if(bc == 2'o2) begin
                  dataout[7:0] <= bufdat[23:16];
                  bc <= bc + 1'b1;
               end
               else if(bc == 2'o3) begin
                  dataout[7:0] <= bufdat[31:24];
                  bc <= bc + 1'b1;              // ... обнулить счетчик байтов, ...
                  bufdat <= txbdata;            // .. данные для передачи в буфер.
               end
            end
         end
         SENDCRC: begin    // Передача контрольной сумм (CRC)
            crcen <= 1'b0;
            if(bc == 2'o0)	begin
               dataout[7:0] <= {~crc[24], ~crc[25], ~crc[26], ~crc[27], ~crc[28], ~crc[29], ~crc[30], ~crc[31]};
               bc <= bc + 1'b1;
            end
            else if(bc == 2'o1) begin
               dataout[7:0] <= {~crc[16], ~crc[17], ~crc[18], ~crc[19], ~crc[20], ~crc[21], ~crc[22], ~crc[23]};
               bc <= bc + 1'b1;
            end
            else if(bc == 2'o2) begin
               dataout[7:0] <= {~crc[8], ~crc[9], ~crc[10], ~crc[11], ~crc[12], ~crc[13], ~crc[14], ~crc[15]};
               bc <= bc + 1'b1;
            end
            else if(bc == 2'o3) begin
               dataout[7:0] <= {~crc[0], ~crc[1], ~crc[2], ~crc[3], ~crc[4], ~crc[5], ~crc[6], ~crc[7]};
               bc <= bc + 1'b1;
               tx_state <= TXDELAY;
            end
         end
         TXDELAY: begin    // Задержка 12 байт и установка сигнала завершения передачи
            txen <= 1'b0;              // Сброс сигнала достоверности данных
            dataout <= 8'hFF;          // Заглушка
            if(i < 11'd12) i <= i + 1'b1; // Таймаут 12 байтов
            else begin
               txdone <= 1'b1;         // 
               tx_state <= WAITDONE;   // ... переход к ожиданию сигнала подтверждения
            end
         end
         WAITDONE: begin   // Возврат в состояние ожидания
            if(txena == 1'b0) begin    // Получен сигнал подтверждения?
               txdone <= 1'b0;         // Да - сброс сигнала завершения ...
               tx_state <= IDLE;       // ... и переход в состояние ожидания
            end
         end
      endcase
   end
end

endmodule
