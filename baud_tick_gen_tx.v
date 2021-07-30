module baud_tick_gen_tx(
    input clk, enable,
    output reg tick);

reg [4:0] cnt = 1'b0;

always @(posedge clk)
    if (enable && cnt <= 5'd23)
        cnt <= cnt + 1'b1;
    else
        cnt <= 3'b0;

always@(posedge clk)
    if(cnt==5'd24)
        tick <= 1'b1;
    else
        tick <= 1'b0;

endmodule
