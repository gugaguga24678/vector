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
