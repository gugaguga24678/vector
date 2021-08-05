module top(
  input clk,
  input RX_sensor,
  output TX_pc,
  input RX_pc,
  input RX_vector,
  output TX_vector
);

wire clk_15m36;

wire rx1_ready;
wire [7:0] rx1_data;
wire rx_idle;
wire rx_eop;

wire wren_1;
wire wren_2;
reg [13:0] rx_cnt_uart = 9'b0;
//reg [23:0] cnt = 24'd0;
reg [15:0] data_in;
wire [13:0] addr_1;
wire [13:0] addr_2;
wire [13:0] addr_1_vector;
wire [13:0] addr_2_vector;

wire [15:0] data_out_1;
wire [15:0] data_out_2;
reg wr_done_355 = 1'b0;

reg tx1_start;
reg [7:0] tx1_data;
wire tx1_busy;

reg [7:0] data_buf;
reg data_flag = 0;
reg data_check_busy = 0;
wire [13:0] tx_cnt_uart = 9'd384;

reg pingpong_flag = 1'b0;
reg [3:0] rx_eop_cnt = 4'b0;

wire wren_1_vector;
wire wren_2_vector;
reg pingpong_flag_vector;

reg [13:0] ram_wr_addr_vector;
wire [13:0] tx_cnt_vector;
reg [13:0] tx_cnt;
reg [13:0] ram_wr_addr_vector_reg;
wire rx_vector_ready;
reg [7:0] ram_data_in;
pll pll_inst(
    .PACKAGEPIN(clk),
    .PLLOUTCORE(clk_15m36),
    .PLLOUTGLOBAL(),
    .RESET(1'b1)
);

uart_rx urx1 (
    .clk(clk_15m36),
    .rx(RX_sensor),
    .rx_ready(rx1_ready),
    .rx_data(rx1_data),
    .rx_idle(rx_idle),
    .rx_eop(rx_eop)
);

SB_SPRAM256KA ramfn_inst1(
    .DATAIN(ram_data_in),
    .ADDRESS(addr_1),
    .MASKWREN(4'b1111),
    .WREN(wren_1),
    .CHIPSELECT(1'b1),
    .CLOCK(clk_15m36),
    .STANDBY(1'b0),
    .SLEEP(1'b0),
    .POWEROFF(1'b1),
    .DATAOUT(data_out_1)
);

SB_SPRAM256KA ramfn_inst2(
    .DATAIN(ram_data_in),
    .ADDRESS(addr_2),
    .MASKWREN(4'b1111),
    .WREN(wren_2),
    .CHIPSELECT(1'b1),
    .CLOCK(clk_15m36),
    .STANDBY(1'b0),
    .SLEEP(1'b0),
    .POWEROFF(1'b1),
    .DATAOUT(data_out_2)
);

uart_tx utx1 (
	.clk(clk_15m36),
	.tx_start(tx1_start),
	.tx_data(tx1_data),
	.tx(TX_pc),
	.tx_busy(tx1_busy)
);

always@(posedge clk_15m36)begin
  if(rx_eop_cnt==4'd8)
    pingpong_flag <= ~pingpong_flag;
  else
    pingpong_flag <= pingpong_flag;
end

assign addr_1 = pingpong_flag ? rx_cnt_uart : tx_cnt_uart;
assign addr_2 = pingpong_flag ? tx_cnt_uart : rx_cnt_uart;

assign wren_1 = pingpong_flag ? rx1_ready : 1'b0;
assign wren_2 = pingpong_flag ? 1'b0 : rx1_ready;

always@(posedge clk_15m36)begin
  if(rx1_ready)
    rx_cnt_uart <= rx_cnt_uart + 1'b1;
  else if(rx_eop_cnt==5'd16)
    rx_cnt_uart <= 10'b0;
  else
    rx_cnt_uart <= rx_cnt_uart;
end

always@(posedge clk_15m36)begin
  if(rx_eop_cnt<5'd16 && rx_eop)
    rx_eop_cnt <= rx_eop_cnt + 1'b1;
  else if(rx_eop_cnt==5'd16)
    rx_eop_cnt <= 1'b0;
  else
    rx_eop_cnt <= rx_eop_cnt;
end

always@(posedge clk_15m36)begin
    ram_data_in <= rx1_data;
end

assign tx_cnt_uart = (tx_cnt <= 'd383) ? tx_cnt : 0;
assign tx_cnt_vector = (tx_cnt <= 'd383) ? 0 : tx_cnt-'d383;

// always@(posedge clk_15m36)begin
//     data_flag <= data_flag_1;
// end

always @(posedge clk_15m36) begin
  // we got a new data strobe
  // let's save it and set a flag
  if(rx_eop_cnt==4'd8)
    tx_cnt <= 10'd0;
  else if(tx_cnt <= 10'd383 + ram_wr_addr_vector_reg)begin
    if(~data_flag) begin
        data_buf <= pingpong_flag ? data_out_2[7:0] : data_out_1[7:0];
        data_flag <= 1;
        data_check_busy <= 1;
    end
    // new data flag is set let's try to send it
    if(data_flag) begin
      // First check if the previous transmission is over
      if(data_check_busy) begin
        if(~tx1_busy) begin
          data_check_busy <= 0;
        end // if(~tx1_busy)
      end else begin // try to send waiting for busy to go high to make sure
        if(~tx1_busy) begin
//          tx1_data <= data_buf;
          tx1_data <= pingpong_flag ? data_out_2[7:0] : data_out_1[7:0];
          tx1_start <= 1'b1;
        end else begin // Yey we did it!
          tx1_start <= 1'b0;
          data_flag <= 0;
          tx_cnt <= tx_cnt + 1'b1;
        end
      end
    end
  end
end

always@(posedge clk_15m36)begin
  if(rx_eop_cnt==5'd16)
    pingpong_flag_vector <= ~pingpong_flag_vector;
  else
    pingpong_flag_vector <= pingpong_flag_vector;
end

assign addr_1_vector = pingpong_flag_vector ? ram_wr_addr_vector : tx_cnt_vector;
assign addr_2_vector = pingpong_flag_vector ? tx_cnt_vector : ram_wr_addr_vector;

assign wren_1_vector = pingpong_flag_vector ? rx_vector_ready : 1'b0;
assign wren_2_vector = pingpong_flag_vector ? 1'b0 : rx_vector_ready;

  
SB_SPRAM256KA vector_ram_1(
   .DATAIN(rx_vector_data),
   .ADDRESS(addr_1_vector),
   .MASKWREN(4'b1111),
   .WREN(wren_1_vector),
   .CHIPSELECT(1'b1),
   .CLOCK(clk_15m36),
   .STANDBY(1'b0),
   .SLEEP(1'b0),
   .POWEROFF(1'b1),
   .DATAOUT(vector_ram_data_out_1)
);

SB_SPRAM256KA vector_ram_2(
   .DATAIN(rx_vector_data),
   .ADDRESS(addr_2_vector),
   .MASKWREN(4'b1111),
   .WREN(wren_2_vector),
   .CHIPSELECT(1'b1),
   .CLOCK(clk_15m36),
   .STANDBY(1'b0),
   .SLEEP(1'b0),
   .POWEROFF(1'b1),
   .DATAOUT(vector_ram_data_out_2)
);
  
uart_rx vector_rx (
    .clk(clk_15m36),
    .rx(RX_vector),
    .rx_ready(rx_vector_ready),
    .rx_data(rx_vector_data),
    .rx_idle(rx_vector_idle),
    .rx_eop(rx_vector_eop)
);

uart_tx vector_tx (
	.clk(clk_15m36),
	.tx_start(tx_vector_start),
	.tx_data(tx_vector_data),
	.tx(TX_vector),
	.tx_busy(tx_vector_busy)
);

uart_rx pc_rx (
    .clk(clk_15m36),
    .rx(RX_pc),
    .rx_ready(rx_pc_ready),
    .rx_data(rx_pc_data),
    .rx_idle(rx_pc_idle),
    .rx_eop(rx_pc_eop)
);

always@(posedge clk_15m36) begin
  if(rx_eop_cnt==5'd16)begin
    ram_wr_addr_vector <= 8'd0;
  end
  else if(rx_vector_ready) begin
    ram_wr_addr_vector <= ram_wr_addr_vector + 1'b1;
  end
  else begin
    ram_wr_addr_vector <= ram_wr_addr_vector;
  end
end

always@(posedge clk_15m36) begin
  if(rx_eop_cnt==5'd16) begin
    ram_wr_addr_vector_reg <= ram_wr_addr_vector;
  end
  else begin
    ram_wr_addr_vector_reg <= ram_wr_addr_vector_reg;
  end
end

always@(posedge clk)begin
    if(rx_pc_data==8'hab)
        vector_start_flag <= 1'b1;
    else if(rx_pc_data==8'hcd)
        vector_stop_flag <= 1'b1;
    else begin
        vector_start_flag <= vector_start_flag;
        vector_stop_flag <= vector_stop_flag; 
    end
end

always@(posedge clk)begin
    
end

endmodule

always@(posedge clk)begin
  case(rx_cnt)
    14'd37,14'd85,14'd133,14'd181,14'd229,14'd277,14'd325,14'd373,14'd421,14'd469,14'd517,14'd565,14'd613,14'd661,14'd709,14'd757:
    14'd38,14'd86,14'd134,14'd182,14'd230,14'd278,14'd326,14'd374,14'd422,14'd470,14'd518,14'd566,14'd614,14'd662,14'd710,14'd758:
    14'd39,14'd87,14'd135,14'd183,14'd231,14'd279,14'd327,14'd375,14'd423,14'd471,14'd519,14'd567,14'd615,14'd663,14'd711,14'd759:
    14'd40,14'd88,14'd136,14'd184,14'd232,14'd280,14'd328,14'd376,14'd424,14'd472,14'd520,14'd568,14'd616,14'd664,14'd712,14'd760:
    14'd41,14'd89,14'd137,14'd185,14'd233,14'd281,14'd329,14'd377,14'd425,14'd473,14'd521,14'd569,14'd617,14'd665,14'd713,14'd761:
    14'd42,14'd90,14'd138,14'd186,14'd234,14'd282,14'd330,14'd378,14'd426,14'd474,14'd522,14'd570,14'd618,14'd666,14'd714,14'd762:
    14'd43,14'd91,14'd139,14'd187,14'd235,14'd283,14'd331,14'd379,14'd427,14'd475,14'd523,14'd571,14'd619,14'd667,14'd715,14'd763:
    14'd44,14'd92,14'd140,14'd188,14'd236,14'd284,14'd332,14'd380,14'd428,14'd476,14'd524,14'd572,14'd620,14'd668,14'd716,14'd764:
    14'd45,14'd93,14'd141,14'd189,14'd237,14'd285,14'd333,14'd381,14'd429,14'd477,14'd525,14'd573,14'd621,14'd669,14'd717,14'd765:
    14'd46,14'd94,14'd142,14'd190,14'd238,14'd286,14'd334,14'd382,14'd430,14'd478,14'd526,14'd574,14'd622,14'd670,14'd718,14'd766:
    14'd47,14'd95,14'd143,14'd191,14'd239,14'd287,14'd335,14'd383,14'd431,14'd479,14'd527,14'd575,14'd623,14'd671,14'd719,14'd767:
    14'd48:
    14'd49:
    14'd50:
    14'd84:
    14'd96:
    14'd97:
    14'd98:
    14'd132:
    14'd144:
    14'd145:
    14'd146:
    14'd180:
    14'd192:
    14'd193:
    14'd194:
    14'd228:
    14'd240:
    14'd241:
    14'd242:
    14'd276:
    14'd288:
    14'd289:
    14'd290:
    14'd324:
    14'd336:
    default:
  endcase
end
