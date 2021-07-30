/*
 *  icebreaker examples - Async uart baud tick generator module
 *
 *  Copyright (C) 2018 Piotr Esden-Tempski <piotr@esden.net>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/***
 * This module generates a bit baud tick multiplied by the oversampling parameter.
 */
module baud_tick_gen(
    input clk, enable,
    output reg tick);

reg [2:0] cnt = 1'b0;

always @(posedge clk)
    if (enable && cnt <= 3'd3)
        cnt <= cnt + 1'b1;
    else
        cnt <= 3'b0;

always@(posedge clk)
    if(cnt==3'd3)
        tick <= 1'b1;
    else
        tick <= 1'b0;

endmodule
