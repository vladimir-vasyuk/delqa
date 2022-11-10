//=================================================================================
// Реализация контроллера Ethernet DELQA
//---------------------------------------------------------------------------------
// Модуль DMA
//=================================================================================
module dma (
   input                clk_i,      // тактовая частота шины
   input                rst_i,      // сброс
// DMA bus
   output reg           dma_req,    // запрос DMA
   input                dma_gnt,    // подтверждение DMA
	output reg[21:0]     dma_adr_o,  // выходной адрес при DMA-обмене
   input     [15:0]     dma_dat_i,  // входная шина данных DMA
   output reg[15:0]     dma_dat_o,  // выходная шина данных DMA
   output reg           dma_stb_o,  // строб цикла шины DMA
   output reg           dma_we_o,   // направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
   input                dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
//
   input                wstart,     // режим передачи
   input                rstart,     // режим приема
   output reg           iocomplete, // признак завершения работы DMA-контроллера
   input     [15:0]     rxdbus,     // входная шина данных
   output reg[15:0]     txdbus,     // выходная шина данных
   input     [10:0]     baddr_i,    // входная шина адреса
   output reg[10:0]     baddr_o,    // выходная шина адреса
   input     [21:1]     haddr,      // входная шина физического адреса
   input     [11:0]     wcount,     // счетчик слов данных
   output reg           nxm         // признак таймаута шины
);

// регистры контроллера DMA
reg  [11:0] data_index;             // указатель текущего слова
reg  [5:0]  bus_wait;               // таймер ожидания ответа при DMA-обмене


// машина состояний контроллера DMA
localparam[3:0] dma_idle = 0; 
localparam[3:0] dma_read_prep = 1;
localparam[3:0] dma_read = 2;
localparam[3:0] dma_read_next = 3;
localparam[3:0] dma_read_done = 4;
localparam[3:0] dma_write_prep = 5; 
localparam[3:0] dma_write = 6;
localparam[3:0] dma_write_next = 7;
localparam[3:0] dma_write_done = 8;
reg  [3:0]  dma_state;


always @(posedge clk_i)  begin
   if (rst_i) begin
   // сброс
      dma_state <= dma_idle; 
      dma_req <= 1'b0; 
      dma_we_o <= 1'b0; 
      dma_stb_o <= 1'b0; 
      nxm <= 1'b0; 
      iocomplete <= 1'b0;
      dma_adr_o <= 22'o0;
      baddr_o <= baddr_i;
   end
      
   // рабочие состояния
   else  begin
      case (dma_state)
         // ожидание запроса
         dma_idle: begin
            nxm <= 1'b0;                                 //  снимаем флаг ошибки nxm
            dma_we_o <= 1'b0;
            dma_adr_o <= 22'o0;
            baddr_o <= baddr_i;
            data_index <= wcount;                        // счетчик слов

            // старт процедуры записи
            if (wstart == 1'b1) begin
               dma_adr_o <= {haddr[21:1], 1'b0};
               dma_req <= 1'b1;                          // поднимаем запрос DMA
//				   data_index <= wcount;                     // счетчик слов
               if (dma_gnt == 1'b1)                      // ждем подтверждения 
                  dma_state <= dma_write_prep;           // и переходим к записи
            end

            // старт процедуры чтения
            else if (rstart == 1'b1) begin
               dma_adr_o <= {haddr[21:1], 1'b0};
               dma_req <= 1'b1;                          // поднимаем запрос DMA
//				   data_index <= wcount;                     // счетчик слов
               if (dma_gnt == 1'b1)                      // ждем подтверждения
                  dma_state <= dma_read_prep;            // и переходим к чтению
            end
            else iocomplete <= 1'b0;
         end

         // чтение данных - подготовка шины к DMA
         dma_read_prep: begin
            dma_we_o <= 1'b0;
            dma_stb_o <= 1'b0;
            bus_wait <= 6'b111111;                       // взводим таймер ожидания шины
            dma_state <= dma_read;                       // переходим к чтению
         end

         dma_read: begin
            dma_dat_o <= rxdbus;                         // выставляем данные
            dma_we_o <= 1'b1;                            // режим записи
            dma_stb_o <= 1'b1;                           // строб транзакции
            bus_wait <= bus_wait - 1'b1;                 // таймер ожидания ответа
            if (|bus_wait == 0) begin
               // таймаут шины
               nxm <= 1'b1;                              // флаг ошибки DMA
               dma_we_o <= 1'b0;
               dma_stb_o <= 1'b0;                        // снимаем строб транзакции
               dma_state <= dma_read_done;               // завершаем чтение
            end
            else if (dma_ack_i == 1'b1) begin
               dma_stb_o <= 1'b0;                        // снимаем строб транзакции
               data_index <= data_index + 1'b1;          // уменьшаем счетчик слов для передачи
               dma_we_o <= 1'b0;
               dma_state <= dma_read_next;
            end
         end

         dma_read_next: begin
//			   dma_we_o <= 1'b0;
            if (|data_index != 0) begin                  // все записано?
               dma_adr_o <=  dma_adr_o +  2'b10;         // увеличиваем физический адрес
               baddr_o <= baddr_o + 1'b1;                // увеличиваем адрес
               dma_state <= dma_read_prep;               // нет - продолжаем
            end
            else
               dma_state <= dma_read_done;               // да - завершаем
         end

         // чтение данных - завершение
         dma_read_done: begin
            dma_req <= 1'b0;                             // освобождаем шину
            if (rstart == 1'b0) begin
               dma_state <= dma_idle;                    // переходим в состояние ожидания команды
               iocomplete <= 1'b0;                       // снимаем подтверждение окончания обмена
            end
            else
               iocomplete <= 1'b1;                       // подтверждаем окончание обмена
         end

         // запись данных - подготовка шины к DMA
         dma_write_prep: begin
            dma_we_o <= 1'b0;
            dma_stb_o <= 1'b1;                           // строб транзакции
            bus_wait <= 6'b111111;                       // взводим таймер ожидания шины
            dma_state <= dma_write;
         end

         // запись данных - обмен по шине
         dma_write: begin
            bus_wait <= bus_wait - 1'b1;                 // таймер ожидания ответа
            txdbus <= dma_dat_i;
            if (|bus_wait == 0) begin
               nxm <= 1'b1;                              // флаг ошибки DMA
               dma_we_o <= 1'b0;
               dma_stb_o <= 1'b0;                        // снимаем строб транзакции
               dma_state <= dma_write_done;              // завершаем запись
            end
            else if (dma_ack_i == 1'b1) begin
               dma_we_o <= 1'b0;
               dma_stb_o <= 1'b0;                        // снимаем строб транзакции
               data_index <= data_index + 1'b1;          // уменьшаем счетчик слов для передачи
               dma_state <= dma_write_next;
            end
         end

         // запись данных - изменение адреса и проверка
         dma_write_next: begin
            if (|data_index != 0) begin                  // все записано?
               dma_adr_o <=  dma_adr_o +  2'b10;         // увеличиваем физический адрес
               baddr_o <= baddr_o + 1'b1;                // увеличиваем адрес
               dma_state <= dma_write_prep;              // нет - продолжаем
            end
            else
               dma_state <= dma_write_done;              // да - завершаем
         end

         // запись данных - завершение
         dma_write_done: begin
            dma_req <= 1'b0;                             // освобождаем шину
            if (wstart == 1'b0)  begin
               iocomplete <= 1'b0;                       // снимаем подтверждение окончания обмена
               dma_state <= dma_idle;                    // переходим в состояние ожидания команды
            end
            else iocomplete <= 1'b1;                     // подтверждаем окончание обмена
         end
      endcase 
   end
end 

endmodule
