//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль CRC
//=================================================================================
module crc(
	input					clk,
	input					reset,
	input [7:0]			data_in,
	input 				enable,
	output reg [31:0]	crc
);
//parameter Tp = 1;

wire [31:0] CrcNext;
wire [7:0] data;

//assign data = enable? {data_in[0],data_in[1],data_in[2],data_in[3],data_in[4],data_in[5],data_in[6],data_in[7]} : {8{1'b0}};
assign data = {data_in[0],data_in[1],data_in[2],data_in[3],data_in[4],data_in[5],data_in[6],data_in[7]};

assign CrcNext[0] = crc[24] ^ crc[30] ^ data[0] ^ data[6];
assign CrcNext[1] = crc[24] ^ crc[25] ^ crc[30] ^ crc[31] ^ data[0] ^ data[1] ^ data[6] ^ data[7];
assign CrcNext[2] = crc[24] ^ crc[25] ^ crc[26] ^ crc[30] ^ crc[31] ^ data[0] ^ data[1] ^ data[2] ^ data[6] ^ data[7];
assign CrcNext[3] = crc[25] ^ crc[26] ^ crc[27] ^ crc[31] ^ data[1] ^ data[2] ^ data[3] ^ data[7];
assign CrcNext[4] = crc[24] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30] ^ data[0] ^ data[2] ^ data[3] ^ data[4] ^ data[6];
assign CrcNext[5] = crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31] ^ data[0] ^ data[1] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
assign CrcNext[6] = crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31] ^ data[1] ^ data[2] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
assign CrcNext[7] = crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31] ^ data[0] ^ data[2] ^ data[3] ^ data[5] ^ data[7];
assign CrcNext[8] = crc[0] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ data[0] ^ data[1] ^ data[3] ^ data[4];
assign CrcNext[9] = crc[1] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ data[1] ^ data[2] ^ data[4] ^ data[5];
assign CrcNext[10] = crc[2] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ data[0] ^ data[2] ^ data[3] ^ data[5];
assign CrcNext[11] = crc[3] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ data[0] ^ data[1] ^ data[3] ^ data[4];
assign CrcNext[12] = crc[4] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ data[0] ^ data[1] ^ data[2] ^ data[4] ^ data[5] ^ data[6];
assign CrcNext[13] = crc[5] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[30] ^ crc[31] ^ data[1] ^ data[2] ^ data[3] ^ data[5] ^ data[6] ^ data[7];
assign CrcNext[14] = crc[6] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30] ^ crc[31] ^ data[2] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
assign CrcNext[15] =  crc[7] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[31] ^ data[3] ^ data[4] ^ data[5] ^ data[7];
assign CrcNext[16] = crc[8] ^ crc[24] ^ crc[28] ^ crc[29] ^ data[0] ^ data[4] ^ data[5];
assign CrcNext[17] = crc[9] ^ crc[25] ^ crc[29] ^ crc[30] ^ data[1] ^ data[5] ^ data[6];
assign CrcNext[18] = crc[10] ^ crc[26] ^ crc[30] ^ crc[31] ^ data[2] ^ data[6] ^ data[7];
assign CrcNext[19] = crc[11] ^ crc[27] ^ crc[31] ^ data[3] ^ data[7];
assign CrcNext[20] = crc[12] ^ crc[28] ^ data[4];
assign CrcNext[21] = crc[13] ^ crc[29] ^ data[5];
assign CrcNext[22] = crc[14] ^ crc[24] ^ data[0];
assign CrcNext[23] = crc[15] ^ crc[24] ^ crc[25] ^ crc[30] ^ data[0] ^ data[1] ^ data[6];
assign CrcNext[24] = crc[16] ^ crc[25] ^ crc[26] ^ crc[31] ^ data[1] ^ data[2] ^ data[7];
assign CrcNext[25] = crc[17] ^ crc[26] ^ crc[27] ^ data[2] ^ data[3];
assign CrcNext[26] = crc[18] ^ crc[24] ^ crc[27] ^ crc[28] ^ crc[30] ^ data[0] ^ data[3] ^ data[4] ^ data[6];
assign CrcNext[27] = crc[19] ^ crc[25] ^ crc[28] ^ crc[29] ^ crc[31] ^ data[1] ^ data[4] ^ data[5] ^ data[7];
assign CrcNext[28] = crc[20] ^ crc[26] ^ crc[29] ^ crc[30] ^ data[2] ^ data[5] ^ data[6];
assign CrcNext[29] = crc[21] ^ crc[27] ^ crc[30] ^ crc[31] ^ data[3] ^ data[6] ^ data[7];
assign CrcNext[30] = crc[22] ^ crc[28] ^ crc[31] ^ data[4] ^ data[7];
assign CrcNext[31] = crc[23] ^ crc[29] ^ data[5];

always @ (posedge clk, posedge reset) begin
	if (reset)
		crc <={32{1'b1}};
	else
		crc <= enable ? CrcNext : crc;
end

endmodule
