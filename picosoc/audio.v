/*
 *  Audio - 1 bit Delta-sigma modulation DAC
 *
fi *  Copyright (C) 2019-2020 René Rebe <rene@exactcode.de>
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
 */

/*
 pc speaker:	1 square wave, 2 levels
 PCjr / Tandy:	3 square waves, 16 levels + noise channel
 GameBlaster	12 square wave, stereo, 4 of which can be used for noise
 OPM		8-channel, 4-operator
 OPL		9 channel, two operators
 OPL2		9 channel, 2 osc, or -3 ch for +5 percussions, 4 (3 new) “sine” waveforms
 OPL3		18 ch, simple stereo, 4 new waveforms, 
 
 adlib:		0x388 Address/Status port, 0x389 Data Write
 PCjr / Tandy:	0xc0 - 0xc8
 
 https://gist.github.com/bryc/e997954473940ad97a825da4e7a496fa
 http://map.grauw.nl/resources/sound/yamaha_ym3526.pdf
 
 12 MHz clk / 250 => 48kHz
 
 sine
 alternating-sine
 "camel"-sine
 square
 logarithmic sawtooth
 
 Attack Rate -> Decay Rate -> Release Rate	
 
 ou should connect the speaker to pin 4 (FPGA pin 19).
 You can use headphones or a small battery-powered speaker.
 Ideally you should connect the speaker with a series resistor and a low-pass filter:
 
         e.g. 470 ohm      eg. 10uF
 AUDIO_O  ---./\/\/\.---o------| |-------> "Analog" output
                        |
                       ---
                       --- eg. 0.1uF
                        |
                        |
                       --- GND
                        -
 # sox some.wav --bits 16 -B --encoding unsigned-integer -c 1 -r 14400 uart.raw
 # dd if=uart.raw bs=1024 count=2048 | xxd -p | fold | sed "s/..../0x&,/g"
*/

module fmop(input clk,
	    input [15:0] fmreg,
	    output reg [15:0] fmval);
   
   always @(posedge clk) begin
      case (fmreg[15:14])
	2'b00: fmval <= fmval + fmreg[13:0]; // square
	2'b01: fmval <= fmval + fmreg[13:0]; // triangle
	2'b10: fmval <= fmval + fmreg[13:0]; // sawtooth
	2'b11: fmval <= fmval + fmreg[13:0]; // noise
      endcase
   end
endmodule

module audio (
   input clk, 
   output dsd,
   output reg word,
   output reg frame,

   input resetn,
   input sel,
   output reg ready,
   input [ 3:0] wstrb,
   input [23:0] addr,
   input [31:0] wdata,
);

   localparam [15:0] clkdiv = 17; // ~= 12MHz / 44100 / 16
   reg [15:0] clk2 = 0;
   reg [3:0] dacbit = 0;
   reg [16:0] pwm = 0; // 1-bit more for overflow / carry
   reg [15:0] dacdata = 16'h7fff;
   reg [15:0] dacnext = 16'h7fff;
   reg 	      dacfree = 1;

   assign dsd = pwm[16]; // directly to overflow / carry
   
   //              EG    x   PG
   //out[15:0] <= envelope * phase;

   
   //   _
   // _| |__   /|/_
   
   // FM operators
 
   reg 	      fm0;
   reg [15:0] fm0frq = 0;
   reg [15:0] fm0vol = 0;
   reg [15:0] fm0cnt = 0;
   wire [15:0] fm0out = (fm0 && fm0frq != 16'b0) ? fm0vol : 16'b0;
   
   reg 	      fm1;
   reg [15:0] fm1frq = 0;
   reg [15:0] fm1vol = 0;
   reg [15:0] fm1cnt = 0;
   wire [15:0] fm1out = (fm1 && fm1frq != 16'b0) ? fm1vol : 16'b0;
   
   reg 	      fm2;
   reg [15:0] fm2frq = 0;
   reg [15:0] fm2vol = 0;
   reg [15:0] fm2cnt = 0;
   wire [15:0] fm2out = (fm2 && fm2frq != 16'b0) ? fm2vol : 16'b0;
   
 /*  
   fmop fm0 (
      .clk(clk),
      .fmreg(fm0reg),
      .fmval(fm0out)
    );

   fmop fm1 (
      .clk(clk),
      .fmreg(fm1reg),
      .fmval(fm1out)
    );
*/

   always @(posedge clk) begin
      clk2 <= clk2 + 1;
      
      if (0) begin
	 // I2S world and frame clock
	 dsd <= clk2[0];
	 if (clk2 == 15)
	   word <= ~word;
	 if (clk2 == 31) begin
	    word <= ~word;
	    frame <= ~frame;
	    clk2 <= 0;
	 end
      end else begin
	 if (clk2 == clkdiv) begin // div for target pwm freq
	    clk2 <= 0;
	    dacbit <= dacbit + 1;

	    // copy fifo for next sample
	    if (dacbit == 15) begin
	       if (fm0cnt != 0)
		 fm0cnt = fm0cnt - 1;
	       else begin
		  fm0 <= ~fm0;
		  fm0cnt <= fm0frq;
	       end
	       if (fm1cnt != 0)
		  fm1cnt <= fm1cnt - 1;
	       else begin
		  fm1 <= ~fm1;
		  fm1cnt <= fm1frq;
	       end
	       if (fm2cnt != 0)
		 fm2cnt <= fm2cnt - 1;
	       else begin
		  fm2 <= ~fm2;
		  fm2cnt <= fm2frq;
	       end
	       
	       // "mix" DAC + FM operators, Note: FM delayed 1 sample!

	       //    _____.      0xffff  0x7fff
	       //    |    |                -0-
	       // ___|    |___ 0         -80000
	       
	       dacdata <= dacnext +
			  ((fm0out + fm1out + fm2out) >> 4);
	       dacfree <= 1;
	    end
	 end
	 
         // 1st order delta sigma accumulation
         pwm <= pwm[15:0] + dacdata;
      end
      
      // mmio system bus interface
      if (resetn) begin
	 ready <= 0;
	 // when selected, wait for free DAC fifo
	 if (sel && dacfree) begin
	    case (addr)
	      24'h00010:
		begin
		   if (wstrb[0]) fm0frq[ 7: 0] <= wdata[ 7: 0];
		   if (wstrb[1]) fm0frq[15: 8] <= wdata[15: 8];
		   if (wstrb[2]) fm0vol[ 7: 0] <= wdata[23:16];
		   if (wstrb[3]) fm0vol[15: 8] <= wdata[31:24];
		end
	      24'h00014:
		begin
		   if (wstrb[0]) fm1frq[ 7: 0] <= wdata[ 7: 0];
		   if (wstrb[1]) fm1frq[15: 8] <= wdata[15: 8];
		   if (wstrb[2]) fm1vol[ 7: 0] <= wdata[23:16];
		   if (wstrb[3]) fm1vol[15: 8] <= wdata[31:24];
		end
	      24'h00018:
		begin
		   if (wstrb[0]) fm2frq[ 7: 0] <= wdata[ 7: 0];
		   if (wstrb[1]) fm2frq[15: 8] <= wdata[15: 8];
		   if (wstrb[2]) fm2vol[ 7: 0] <= wdata[23:16];
		   if (wstrb[3]) fm2vol[15: 8] <= wdata[31:24];
		end

	      //24'h00004: // NYI: right DAC
	      default: // left DAC
		begin
		   if (wstrb[0]) dacnext[ 7: 0] <= wdata[ 7: 0];
		   if (wstrb[1]) dacnext[15: 8] <= wdata[15: 8];
		   dacfree <= 0;
		end
	    endcase
	    ready <= 1;
	 end
      end
   end

endmodule
