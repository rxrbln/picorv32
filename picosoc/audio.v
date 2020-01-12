/*
 pc speaker:	1 square wave, 2 levels
 PCjr / Tandy:	3 square waves, 16 levels + noise channel
 OPL			
 OPL2		9 channels, 2 osc, or -3 ch for +5 percussions, 4 “sine” waveforms
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
 # sox some.wav --bits 8 --encoding unsigned-integer -c 1 -r 14400 uart.raw
*/

module audio (
   input clk, 
   output dsd,
   output reg word,
   output reg frame,

   input resetn,
   input        sel,
   input [ 3:0] wstrb,
   input [23:0] addr,
   input [31:0] wdata,
);

   localparam [15:0] clkdiv = 17; // ~= 12MHz / 44100 / 16
   reg [15:0] clk2 = 0;
   reg [3:0] dacbit = 0;
   reg [16:0] pwm = 0; // 1-bit more for overflow / carry
   reg [15:0] dacdata = 16'h7fff;

   assign dsd = pwm[16]; // directly to overflow / carry
   
   //              EG    x   PG
   //out[15:0] <= envelope * phase;

   
   //   _
   // _| |__   /|/_

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
	    //if (dacbit == 4'hf) dacdada <= dacdata + 2300; // test inc only
	    
	    // 1st order delta sigma accumulation
	    pwm <= pwm[15:0] + dacdata + 1;
	 end
      end
   end 

   // mmio system bus interface
   always @(posedge clk) begin
      if (resetn) begin
	 if (sel) begin
	    if (wstrb[0]) dacdata[ 7: 0] <= wdata[ 7: 0];
	    if (wstrb[1]) dacdata[15: 8] <= wdata[15: 8];
	 end
      end
   end

endmodule
