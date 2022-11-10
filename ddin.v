module ddin(
   input          rxclk_i,       // Синхросигнал
   input          rst,           // Сигнал сброса
   input          rxdv,          // Сигнал готовности данных
   input  [3:0]   dat_i,         // Шина входных данных
   output [7:0]   dat_o,         // Шина выходных данных
   output         rxclk_o,       // Новый синхросигнал
   output         rxdv_o,        // Новый сигнал готовности данных блока приема
   output         rxer_o         // Сигнал ошибки выходных данных
);

reg  [3:0]  lsb;                 // Буфер младших разрядов
reg  [7:0]  rxbufd[0:3];         // Кольцевой буфер
reg  [1:0]  raddr, waddr;        // Аоресные регистры
reg         rxclk_reg = 1'b0;    // Регистр генерации синхросигнала
reg         eo = 1'b0;           // Чет-нечет счетчик
reg         wr_err = 1'b0;       // Сигнал ошибки
reg  [7:0]  dato;                // Буфер выходных данных
reg         rxdvn;

wire [1:0]  raddrl;
assign raddrl = raddr -1'b1;     // Указатель на хвост буфера
assign dat_o = dato;
assign rxclk_o = rxclk_reg;
assign rxdv_o = rxdvn;
assign rxer_o = wr_err;

// Генерация нового синхросигнала
always @(posedge rxclk_i, posedge rst) begin
   if(rst)  rxclk_reg <= 1'b0;
   else     rxclk_reg <= rxclk_reg + 1'b1;
end

// Чтение 2-х полубайт и запись в кольцевой буфер
always @(posedge rxclk_i, posedge rst) begin
   if(rst) begin
      eo <= 1'b0;
      waddr <= 1'b0; wr_err <= 1'b0;
   end
   else begin
      if(rxdv) begin                      // Данные готовы?
         if(eo == 1'b0) begin             // Да, Чет?
            lsb <= dat_i; eo <= 1'b1;     // Сохраняем и переключаем на нечет
         end
         else begin                       // Нечет
            rxbufd[waddr][7:0] <= {dat_i[3:0],lsb[3:0]}; // Запись в кольцевой буфер
            if(~wr_err) begin             // Ошибка?
               if(waddr != raddrl)        // Нет. Конец буфера?
                  waddr <= waddr + 1'b1;  // Нет - инкремент
               else
                  wr_err <= 1'b1;         // Конец буфера, установить сигнал ошибки
            end
            eo <= 1'b0;                   // Переключаем на чет
         end
      end
   end
end

// Чтение кольцевого буфера
always @(posedge rxclk_o, posedge rst) begin
   if(rst) begin
      dato <= 8'o0; raddr <= 1'b0; rxdvn <= 1'b0;
   end
   else begin
      if(raddr != waddr) begin            // Конец буфера?
         dato[7:0] <= rxbufd[raddr][7:0]; // Нет - данные в выходной буфер ...
         raddr <= raddr + 1'b1;           // ... и инкремент адреса
         rxdvn <= 1'b1;
      end
      else
         rxdvn <= 1'b0;
   end
end

endmodule
