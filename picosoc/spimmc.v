/*
 *  SPI MMC memory interface, based on PicoSoC SPI
 *  Copyright (C) 2020  René Rebe <rene@exactcode.de>
 *
 *  PicoSoC - A simple example SoC using PicoRV32
 *
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

module spimmc (
	input clk, reset,
	input valid,
	output reg ready,
	input [31:0] wdata,
	input [7:0] wdata_cnt,
	output [31:0] rdata,

	output reg spi_csn,
	output spi_sclk,
	output reg spi_mosi,
	input spi_miso,

	// BEGIN Logic Analizer
	input log_sel,
	input [31:0] log_addr,
        output reg [31:0] log_rdata,
	output reg log_ready,
);
   
   	parameter integer LOG_SIZE = 2048;
        reg [3:0] log_buffer[LOG_SIZE];
        reg [10:0] log_pos = 0;

        reg _spi_sclk;
        assign spi_sclk = _spi_sclk; // positive or negative CPOL
  
   
   	always @(posedge clk) begin
	   log_ready <= 0;
	   if (log_sel) begin
	      if (log_addr[31:2] < LOG_SIZE)
		log_rdata <= {28'h0, log_buffer[log_addr[12:2]]};
	      else
		log_rdata <= {16'h0, log_pos};
	      log_ready <= 1;
	   end
	end
        // END Logic Analyzer
   
        reg [31:0] buffer;
	reg [7:0] xfer_cnt;
	reg [3:0] state;
   
        assign rdata = buffer;
   
        reg [9:0] div = 200;
        reg [9:0] clkcnt = 0;

        reg sample;
   
	always @(posedge clk) begin
	   ready <= 0;
	   
	   if (reset || !valid || ready) begin
	      if (reset) begin
		 spi_csn <= 1;
		 state <= reset ? 0 : 1; // reset to init state 0
	      end
	      spi_mosi <= 0; // just signal prettification
	      _spi_sclk <= 1;
	      xfer_cnt <= 0;
	   end else begin
	      // divide clock (e.g. 40 MHz -> 400kHz
	      clkcnt <= clkcnt + 1;
	      
	      if (sample && (clkcnt == div || clkcnt == div / 2)) begin // Logic Analyzer
		 log_buffer[log_pos] <= {spi_miso, spi_mosi, spi_csn, spi_sclk};
		 if (log_pos < LOG_SIZE - 1)
		   log_pos <= log_pos + 1;
		 else
		   sample <= 0;
	      end
	      
	      if (clkcnt == div) begin
		 clkcnt <= 0;
		 
		 sample = 1; // Logic Analizer start sampling after 1st valid select
		 
		 // update sclk one clk, phase delayed
		 //spi_sclk <= !_spi_sclk;
		 
		 // current transfer
		 if (xfer_cnt) begin
		    if (_spi_sclk) begin
		       _spi_sclk <= 0;
		       spi_mosi <= buffer[31];
		    end else begin
		       _spi_sclk <= 1;
		       buffer <= {buffer, spi_miso};
		       xfer_cnt <= xfer_cnt - 1;
		    end
		 end else // xfer end, transition to next state
		   case (state)
		     0: begin // initial min 74 cycles clk w/o cs
			buffer <= 32'hffff_ffff;
			xfer_cnt <= 80;
			state <= 1;
		     end
		     1: begin // start write out
			if (wdata_cnt == 255) begin // OOB chip unselect
			   spi_csn <= 1;
			   ready <= 1;
			end else begin
			   spi_csn <= 0; // active command read/write
			   buffer <= wdata;
			   xfer_cnt <= wdata_cnt;
			   state <= 2;
			end
		     end
		     2: begin // start read, always try 8
			xfer_cnt <= wdata_cnt == 0 ? 8 : 0; // read or write cycle?
			buffer <= 32'hffff_ffff; // keep output high
			state <= 3;
		     end
		     3: begin
			state <= 1;
			ready <= 1;
		     end
		   endcase // case (state)
	      end // if (clkcnt == div)
	   end // else: !if(reset || !valid || ready)
	end // always @ (posedge clk)
   
endmodule // spimmc

