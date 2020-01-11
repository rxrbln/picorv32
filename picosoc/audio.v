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
*/

module audio (
   input clk, 
   output reg dsd,
   output reg word,
   output reg frame,
);

   //localparam [15:0] oversample = 250;
   reg [15:0] cnt = 0;
   reg [15:0] pwm = 8'b11110010;
   reg [31:0] v = 0;

   //              EG    x   PG
   //out[15:0] <= envelope * phase;

   //   _
   // _| |__   /|/_

   always @(posedge clk) begin
      cnt <= cnt + 1;

      /*
      // I2S world and frame clock
      dsd <= cnt[0];
      if (cnt == 15)
	word <= ~word;
      
      if (cnt == 31) begin
	 word <= ~word;
	 frame <= ~frame;
	 cnt <= 0;
      end
       */
      
      if (cnt[1:0] == 3) begin

	 // generate cheap signal
	 if (cnt[1:0] == 3) begin
	    v <= v + 1;
	 end

	 // map to digital bitstream
	 case (v[3:0])
	   4'h0 : pwm <= 16'b0000000000000000;
	   4'h1 : pwm <= 16'b0000000000000001;
	   4'h2 : pwm <= 16'b0000000000000011;
	   4'h3 : pwm <= 16'b0000000000000111;
	   4'h4 : pwm <= 16'b0000000000001111;
	   4'h5 : pwm <= 16'b0000000000011111;
	   4'h6 : pwm <= 16'b0000000000111111;
	   4'h7 : pwm <= 16'b0000000011111111;
	   4'h8 : pwm <= 16'b0000000111111111;
	   4'h9 : pwm <= 16'b0000001111111111;
	   4'ha : pwm <= 16'b0000011111111111;
	   4'hb : pwm <= 16'b0000111111111111;
	   4'hc : pwm <= 16'b0001111111111111;
	   4'hd : pwm <= 16'b0011111111111111;
	   4'he : pwm <= 16'b0111111111111111;
	   4'hf : pwm <= 16'b1111111111111111;
	 endcase
	 
	 dsd <= pwm[15 - cnt[5:2]];
      end
   end
endmodule
