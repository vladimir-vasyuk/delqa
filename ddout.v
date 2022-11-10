module ddout(
   input          txclk_i,       // Синхросигнал выходных данных
   input          rst,           // Сигнал сброса
   input          txen_i,        // Сигнал готовности входных данных
   input  [7:0]   dat_i,         // Шина входных данных
   output [3:0]   dat_o,         // Шина выходных данных
   output         txclk_o,       // Синхросигнал входных данных
   output reg     txen_o,        // Сигнал готовности выходных данных
   output         txerr_o        // Сигнал ошибки выходных данных
);

reg  [7:0]  txbufd[0:3];         // Кольцевой буфер
reg  [1:0]  raddr, waddr;        // Аоресные регистры
reg         eo = 1'b0;           // Чет-нечет счетчик
reg         wr_err = 1'b0;       // Сигнал ошибки
reg  [3:0]  dato;                // Буфер выходных данных
reg         txclk_reg = 1'b0;    // Регистр генерации синхросигнала

wire [1:0]  raddrl;
assign raddrl = raddr -1'b1;     // Указатель на хвост буфера
assign dat_o = dato;
assign txerr_o = wr_err;

// Генерация нового синхросигнала
always @(posedge txclk_i, posedge rst) begin
   if(rst)  txclk_reg <= 1'b0;
   else     txclk_reg <= txclk_reg + 1'b1;
end
assign txclk_o = txclk_reg;

always @(negedge txclk_o, posedge rst) begin
   if(rst) begin
      waddr <= 2'b0; wr_err <= 1'b0;
   end
   else if(txen_i) begin
      txbufd[waddr][7:0] <= dat_i;
      if(~wr_err) begin             // Ошибка?
         if(waddr != raddrl)        // Нет. Конец буфера?
            waddr <= waddr + 1'b1;  // Нет - инкремент
         else
            wr_err <= 1'b1;         // Конец буфера, установить сигнал ошибки
      end
   end
end

always @(negedge txclk_i, posedge rst) begin
   if(rst) begin
      eo <= 1'b0; txen_o <= 1'b0;
      raddr <= 2'b0; dato <= 4'o0;
   end
   else begin
      if(raddr != waddr) begin            // Конец данных?
         txen_o <= 1'b1;                  // Нет, установить сигнал готовности выходных данных
         if(eo == 1'b0) begin             // Если чет ...
            dato[3:0] <= txbufd[raddr][3:0]; // ... вывод младшего полубайта ...
            eo <= 1'b1;                   // ... и переключаем на нечет
         end
         else begin                       // Нечет ...
            dato[3:0] <= txbufd[raddr][7:4]; // ... вывод старшего полубайта, ...
            raddr <= raddr + 1'b1;        // ... инкремент адреса ...
            eo <= 1'b0;                   // ... и переключаем на чет
         end
      end                                 // Конец данных ...
      else txen_o <= 1'b0;                // ... сброс сигнала готовности выходных данных
   end
end

endmodule
