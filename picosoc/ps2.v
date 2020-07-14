/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2020  René Rebe <rene@exactcode.de>
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
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

// based on work by Paul Ruiz and Grant Searle

module ps2 #(parameter integer DEFAULT_DIV = 1) (
	input clk,
	input resetn,

	input ps2_clk,
	input ps2_data,

	input         reg_dat_re,
	output [31:0] reg_dat_do,
	output        reg_dat_wait,
);
   
   reg [7:0] 	      recv_buf_data;
   reg 		      recv_buf_valid;
   
   assign reg_dat_do = recv_buf_valid ? recv_buf_data : ~0;

   // sync ps2_data
   //
   reg 		      serin;
   always @(posedge clk)
     serin <= ps2_data;
   
   // sync & 'debounce' ps2_clock
   //
   parameter LEN = 8;
   reg 		      bitclk = 0;
   reg [LEN:0] 	      stable = 0;
   
   always @(posedge clk)
     begin
	stable = {stable[LEN-1:0], ps2_clk};
	if ( &stable) bitclk <= 1;
	if (~|stable) bitclk <= 0;
     end
   
   wire bitedge = bitclk && (~|stable[LEN-1:0]);
   
   // clock in KBD bits (start - 8 data - odd parity - stop)
   //
   reg [8:0] shift = 0;
   reg [3:0] bitcnt = 0;
   reg 	     parity = 0;
   
   always @(posedge clk)
     if (!resetn) begin
	recv_buf_data <= 0;
	recv_buf_valid <= 0;
     end else begin
	if (reg_dat_re)
	  recv_buf_valid <= 0;
	
	//strobe <= 0; err <= 0;
	if (bitedge) begin
	   // wait for start bit
	   if (bitcnt == 0) begin
              parity <= 0;
              if (!serin)
		bitcnt <= bitcnt + 1;
           end
	   // shift in 9 bits (8 data + parity)
	   else if (bitcnt < 10) begin
              shift  <= {serin, shift[8:1]};
              parity <= parity ^ serin;
              bitcnt <= bitcnt + 1;
           end
	   // check stop bit, parity
	   else begin
              bitcnt <= 0;
              if (parity && serin) begin
		 recv_buf_data <= shift[7:0];
		 recv_buf_valid <= 1;
              end
              //else err <= 1;
	   end
	end
     end // always @ (posedge clk)
endmodule
