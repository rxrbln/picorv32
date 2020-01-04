/*
 *  VGA - A simple Video Graphic Array
 *
 *  Copyright (C) 2019-2020 René Rebe <rene@exactcode.de>
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

/*
 * Currently mostly memory limited and proof of concept.
 * (Oh, and also some pixel here and there off-by-one.)
 * 640x480, 8x16 text + color attribute & palette.
 */

// TODO: parameterize width, and size
module dpram (
   input clk, wen, ren,
   input [11:0] waddr, raddr,
   input [15:0] wdata,
   output reg [15:0] rdata
);
   reg [15:0] mem [0:4096-1];
   always @(posedge clk) begin
      if (wen)
        mem[waddr] <= wdata;
      if (ren)
        rdata <= mem[raddr];
   end
endmodule

module vga(
   input clk,
   input resetn,
   input pixclk,
   
   input        sel,
   output       ready,
   input [ 3:0] wstrb,
   input [23:0] addr,
   input [31:0] wdata,
   output [31:0] rdata,
   
   output P1A1,
   output P1A2,
   output P1A3,
   output P1A4,
   output P1A7,
   output P1A8,
   output P1A9,
   output P1A10,

   output P1B1,
   output P1B2,
   output P1B3,
   output P1B4,
   output P1B7,
   output P1B8,
   output P1B9,
   output P1B10,
);

   ////////////////////////////
   // pmod
   
   assign P1A1  = vid_r[7];
   assign P1A2  = vid_r[5];
   assign P1A3  = vid_g[7];
   assign P1A4  = vid_g[5];
   assign P1A7  = vid_r[6];
   assign P1A8  = vid_r[4];
   assign P1A9  = vid_g[6];
   assign P1A10 = vid_g[4];
   
   assign P1B1  = vid_b[7];
   assign P1B2  = vid_clk;
   assign P1B3  = vid_b[4];
   assign P1B4  = vid_hs;
   assign P1B7  = vid_b[6];
   assign P1B8  = vid_b[5];
   assign P1B9  = vid_de;
   assign P1B10 = vid_vs;
   
   ////////////////////////////
   // clocking


   // DDR output buffer to repeat pixel clock
   wire    vid_clk;
   SB_IO #(
      // DDR output, regular input
      .PIN_TYPE(6'b010001)
   ) pixclk_buf (
      .PACKAGE_PIN(vid_clk),
      .LATCH_INPUT_VALUE(1'b0),
      .CLOCK_ENABLE(1'b1),
      .INPUT_CLK(pixclk),
      .OUTPUT_CLK(pixclk),
      .OUTPUT_ENABLE(1'b1),
      .D_OUT_0(1'b1),
      .D_OUT_1(1'b0)
   );

   ////////////////////////////
   // mmio bus buffer interface
   reg [31:0] vga_rdata;
   reg 	      vga_ready;
   assign ready = vga_ready;
   assign rdata = vga_rdata;
   
   ////////////////////////////
   // video timing
   
   wire    hsync;
   wire    vsync;
   wire    data_en;
   
   video_timing video_timing_inst (
      .clk(pixclk),
      .hsync(hsync),
      .vsync(vsync),
      .data_en(data_en)
   );

   ////////////////////////////
   // output & test pattern
   
   reg [7:4] vid_r;
   reg [7:4] vid_g;
   reg [7:4] vid_b;
   reg 	     vid_hs;
   reg 	     vid_vs;
   reg 	     vid_de;
   
   reg [8:0]   frame = 0;
   reg [15:0]  xpos = 0;
   reg [15:0]  ypos = 0;
   reg [18:0]  ipos = 0;
   
   reg 	       hsync_prev = 0;
   reg 	       vsync_prev = 0;
   reg 	       hsync_pulse = 0;
   reg 	       vsync_pulse = 0;
   
   
   // ReneR video core
   /*
   reg [7:0]   font[0:8*1024-1]; // 8k font / 2nd half fb
   initial $readmemh("charset.hex", font, 0);
    */
   // initial font[0] = 8'b00000000; initial font[1] = 8'b11111000;
   // ...

   reg 	vramwen = 0;
   reg 	vramren = 0;
   wire [15:0] vramrdata;
   reg [15:0] vramwdata;
   reg [11:0] vramwaddr;
   reg [11:0] vramraddr;
   
   dpram nvram (
      .clk(pixclk),
      .ren(vramren),
      .wen(vramwen),
      .raddr(vramraddr),
      .waddr(vramwaddr),
      .rdata(vramrdata),
      .wdata(vramwdata)
    );

   reg 	fontwen = 0;
   reg 	fontren = 0;
   wire [15:0] fontrdata;
   reg [15:0] fontwdata;
   reg [11:0] fontwaddr;
   reg [11:0] fontraddr;
   
   dpram fontram (
      .clk(pixccclk),
      .ren(fontren),
      .wen(fontwen),
      .raddr(fontraddr),
      .waddr(fontwaddr),
      .rdata(fontrdata),
      .wdata(fontwdata)
    );

   /*
   reg [15:0]  vram[0:4*1024-1]; // 8k text+attribute / frame buffer
   initial vram[512+0] = "\001H";
   initial vram[512+1] = "\002E";
   initial vram[512+2] = "\003L";
   initial vram[512+3] = "\004L";
   initial vram[512+4] = "\005O";
   initial vram[512+5] = "\000 ";
   initial vram[512+6] = "\006F";
   initial vram[512+7] = "\007P";
   initial vram[512+8] = "\010G";
   initial vram[512+9] = "\011A";
   initial vram[512+10] = "\012!";
   initial vram[512+11] = "\013 ";
   initial vram[512+12] = "\014&";
   initial vram[512+13] = "\015Y";
   initial vram[512+14] = "\014T";
   initial vram[512+15] = "\013;";
   initial vram[512+16] = "\012-";
   initial vram[512+17] = "\011)";
   initial vram[512+78] = "\100X";
   initial vram[512+77] = "\100X";
   initial vram[512+78] = "\100Y";
   initial vram[512+79] = "\100Z";

   initial vram[128] = "\020@";
   initial vram[129] = "\040b";
   initial vram[130] = "\060c";
   initial vram[131] = "\1001";
   initial vram[132] = "\1202";
   initial vram[133] = "\1403";
   initial vram[134] = "\160?";

   initial vram[256+0] = "\001H";
   initial vram[256+1] = "\002E";
   initial vram[256+2] = "\003L";
   initial vram[256+3] = "\004L";
   initial vram[256+4] = "\005O";
   initial vram[256+5] = "\000 ";
   initial vram[256+6] = "\006F";
   initial vram[256+7] = "\007P";
   initial vram[256+8] = "\010G";
   initial vram[256+9] = "\011A";
   initial vram[256+10] = "\012!";
   initial vram[256+11] = "\013 ";
   initial vram[256+12] = "\014&";
   initial vram[256+13] = "\015Y";
   initial vram[256+14] = "\014T";
   initial vram[256+15] = "\013;";
   initial vram[256+16] = "\012-";
   initial vram[256+17] = "\011)";

   initial vram[384+0] = "\101H";
   initial vram[384+1] = "\102E";
   initial vram[384+2] = "\103L";
   initial vram[384+3] = "\104L";
   initial vram[384+4] = "\105O";
   initial vram[384+5] = "\100 ";
   initial vram[384+6] = "\106F";
   initial vram[384+7] = "\107P";
   initial vram[384+8] = "\110G";
   initial vram[384+9] = "\111A";
   initial vram[384+10] = "\112!";
   initial vram[384+11] = "\113 ";
   initial vram[384+12] = "\114&";
   initial vram[384+13] = "\115Y";
   initial vram[384+14] = "\314T";
   initial vram[384+15] = "\313;";
   initial vram[384+16] = "\112-";
   initial vram[384+17] = "\111)";

   initial vram[128*30 - 3] = "\61A";
   initial vram[128*30 - 2] = "\101B";
   initial vram[128*30 - 1] = "\141C";
   
   initial $readmemh("vram16.hex", vram);
   */
    
   // text or graphic mode?
   /*localparam*/ reg textmode = 1;
   
   reg [7:0] attr;
   reg [7:0]   row;
   reg [7:0]   nrow;
   reg [7:0]   mask;
   
   reg [11:0]  taddr; // text
   reg [11:0]  faddr; // font
   
   reg [3:0]   col;
   reg [3:0]   fgcol;
   reg [3:0]   bgcol;
   reg blink;
   
   reg [23:0]  pal0 = 24'h000000; // black
   reg [23:0]  pal1 = 24'h0000aa; // blue
   reg [23:0]  pal2 = 24'h00aa00; // green
   reg [23:0]  pal3 = 24'h00aaaa; // cyan
   reg [23:0]  pal4 = 24'haa0000; // red
   reg [23:0]  pal5 = 24'haa00aa; // magenta
   reg [23:0]  pal6 = 24'haaaa00; // brown !!
   reg [23:0]  pal7 = 24'haaaaaa; // light gray
   reg [23:0]  pal8 = 24'h555555; // "light black" / dark gray
   reg [23:0]  pal9 = 24'h5555ff; // light blue
   reg [23:0]  pal10 = 24'h55ff55; // light green
   reg [23:0]  pal11 = 24'h55ffff; // light cyan
   reg [23:0]  pal12 = 24'hff5555; // light red
   reg [23:0]  pal13 = 24'hff55ff; // light magenta
   reg [23:0]  pal14 = 24'hffff55; // yellow
   reg [23:0]  pal15 = 24'hffffff; // white
   
   reg [23:0]  fb_rgb;
   
   //                    // mask             color
   reg [31:0]  cursor0  = 32'b10000000000000000000000000000000; // cursor planes
   reg [31:0]  cursor1  = 32'b11000000000000000100000000000000;
   reg [31:0]  cursor2  = 32'b11100000000000000110000000000000;
   reg [31:0]  cursor3  = 32'b11110000000000000111000000000000;
   reg [31:0]  cursor4  = 32'b11111000000000000111100000000000;
   reg [31:0]  cursor5  = 32'b11111100000000000111110000000000;
   reg [31:0]  cursor6  = 32'b11111110000000000111111000000000;
   reg [31:0]  cursor7  = 32'b11111111100000000111111100000000;
   reg [31:0]  cursor8  = 32'b11111111110000000111111110000000;
   reg [31:0]  cursor9  = 32'b11111111111000000111111111000000;
   reg [31:0]  cursor10 = 32'b11111111000000000111111000000000;
   reg [31:0]  cursor11 = 32'b11100111100000000110011000000000;
   reg [31:0]  cursor12 = 32'b10000011110000000000001100000000;
   reg [31:0]  cursor13 = 32'b00000011110000000000001100000000;
   reg [31:0]  cursor14 = 32'b00000001111000000000000110000000;
   reg [31:0]  cursor15 = 32'b00000000110000000000000000000000;
   
   reg [31:0]  curs_row;
   
   reg [15:0]  cursx = 16'd80; // cursor bottom, right x/y
   reg [15:0]  cursy = 16'd70;

   reg [15:0]  cursix = 16'd40; // current cursor index
   reg [15:0]  cursiy = 16'd50;
   
   reg [23:0]  curspal0 = 24'h000000; // cursor palette
   reg [23:0]  curspal1 = 24'hffffff;
   reg cursp0;
   reg cursp1;
   
   
   // ----------------------------------------------------------------------------
   // Video output
   // ----------------------------------------------------------------------------
   
   
   always @(posedge pixclk) begin
      hsync_prev <= hsync;
      vsync_prev <= vsync;
      hsync_pulse <= hsync & ~hsync_prev;
      vsync_pulse <= vsync & ~vsync_prev;
      
      frame <= frame + vsync_pulse;
      
      xpos <= hsync_pulse ? 0 : xpos + data_en;
      ypos <= vsync_pulse ? 0 : ypos + hsync_pulse;
      ipos <= vsync_pulse ? 0 : ipos + data_en;
      
      // text & graphic pixel generation
      if (1) begin
	 if (textmode) begin
	    // text mode, pre-load every 8 pixels
	    if (!data_en || xpos[2:0] == 2) begin
	       // load char index from vram, * 256 bytes, 128 "words"  per row
	       // interleaved text color attribute
	       taddr <= ((ypos & 16'hFFF0) << 3) | (data_en ? (xpos[10:3] + 1) : 0);
	       
	       vramren <= 1;
	       vramraddr <= taddr;
	       
	       //nchar <= vram[taddr][7:0];
	       //nattr <= vram[taddr][15:8];
	    end
	    
	    if (!data_en || xpos[2:0] == 6) begin
	       faddr <= (vramrdata[7:0] << 4) | ypos[3:0];
	       fontren <= 1;
	       fontraddr <= faddr[11:0];
	    end
	    if (!data_en || xpos[2:0] == 7) begin
	       fontren <= 0;
	       nrow <= fontrdata[7:0];
	    end
	    
	    if (xpos[2:0] == 0) begin
	       // transfer pre-loaded at begin of each pixel
	       attr <= vramrdata[15:8];
	       row <= nrow;
	    end
	    
	    // text mode, only load every 8 pixels
	    if (1) begin
	       fgcol <= attr[3:0];
	       bgcol <= attr[6:4];
	       blink <= attr[7:7];
	    end else begin
	       fgcol <= 24'hff0000;
	       bgcol <= 24'haaaaaa;
	    end

	    begin
	       mask <= 8'b10000000 >> xpos[2:0];
	       if (row & mask && (!blink || frame & 5'b10000)) begin
		  col <= fgcol;
	       end else begin
		  col <= bgcol;
	       end
	    end

	 end else begin // if (textmode)
	    // graphic mode, "double scan" 320x240 => 640x480
	    begin
	       // 2x scale, 4 of 16 bits
	       col <= 0; //vram[ipos[17:0] >> 4] >> (ipos[3:0] & 4'b1100);
	    end
	 end
	 
	 case (col)
	   4'h1 : fb_rgb <= pal1;
	   4'h2 : fb_rgb <= pal2;
	   4'h3 : fb_rgb <= pal3;
	   4'h4 : fb_rgb <= pal4;
	   4'h5 : fb_rgb <= pal5;
	   4'h6 : fb_rgb <= pal6;
	   4'h7 : fb_rgb <= pal7;
	   4'h8 : fb_rgb <= pal8;
	   4'h9 : fb_rgb <= pal9;
	   4'ha : fb_rgb <= pal10;
	   4'hb : fb_rgb <= pal11;
	   4'hc : fb_rgb <= pal12;
	   4'hd : fb_rgb <= pal13;
	   4'he : fb_rgb <= pal14;
	   4'hf : fb_rgb <= pal15;
	   default : fb_rgb <= pal0;
	 endcase

	 // cursor overlay
	 cursix <= xpos + 16 - cursx; // 1 .. 16
	 cursiy <= ypos + 16 - cursy;
	 if (cursix && cursiy && cursix <= 16 && cursiy <= 16) begin
	    case (cursiy)
	      4'h1 : curs_row <= cursor0;
	      4'h2 : curs_row <= cursor1;
	      4'h3 : curs_row <= cursor2;
	      4'h4 : curs_row <= cursor3;
	      4'h5 : curs_row <= cursor4;
	      4'h6 : curs_row <= cursor5;
	      4'h7 : curs_row <= cursor6;
	      4'h8 : curs_row <= cursor7;
	      4'h9 : curs_row <= cursor8;
	      4'ha : curs_row <= cursor9;
	      4'hb : curs_row <= cursor10;
	      4'hc : curs_row <= cursor11;
	      4'hd : curs_row <= cursor12;
	      4'he : curs_row <= cursor13;
	      4'hf : curs_row <= cursor14;
	      default : curs_row <= cursor15;
	    endcase // case (ypos - cursy)
	    
	    // transparent, invert, plane0, plane1?
	    cursp0 <= curs_row[31:16] >> (16 - cursix);
	    cursp1 <= curs_row[15:0] >> (16 - cursix);
	    if (cursp0) begin
	       if (cursp1)
		 {vid_r, vid_g, vid_b} <= curspal1;
	       else
		 {vid_r, vid_g, vid_b} <= curspal0;
	    end else begin
	       if (cursp1) // invert / "highlight"
		 {vid_r, vid_g, vid_b} <= ~fb_rgb; //{8'hff - fb_rgb[23:16], 8'hff - fb_rgb[15:8], 8'hff - fb_rgb[7:0]}
	       else
		 {vid_r, vid_g, vid_b} <= fb_rgb;
	    end
	 end else begin
	    {vid_r, vid_g, vid_b} <= fb_rgb;
	 end
	 
      end else begin 
	 if (xpos == frame || ypos == frame) begin
	    //vga_rgb[23:0] <= {24'b11111111111111111111111};
	    vid_r[7:4] <= 4'b1111;
	    vid_g[7:4] <= 4'b1111;
	    vid_b[7:4] <= 4'b1111;
	 end else begin
	    //vga_rgb[23:0] <= {xpos[11:0], ypos[11:0]};
	    vid_r[7:4] <= ypos + frame;
	    vid_g[7:4] <= xpos + frame;
	    vid_b[7:4] <= xpos + ypos +frame;
	 end
      end
      
      vid_hs <= hsync_prev;
      vid_vs <= vsync_prev;
      vid_de <= data_en;
   end

   
   // mmio system bus interface
   always @(posedge clk) begin
      if (resetn) begin
	 vga_ready <= 0;
	 vramwen <= 0;
	 fontwen <= 0;
	 //vramren <= 0;
	 
	 if (sel) begin
	    // must be aligned 16 bit writes, ..!
	    if (addr < 24'h400000) begin // VRAM
	       if (wstrb[3:0] != 4'b0) begin
		  vramwen <= 1;
		  vramwaddr <= addr[13:2];
		  if (wstrb[0]) vramwdata[ 7: 0] <= wdata[ 7: 0];
		  if (wstrb[1]) vramwdata[15: 8] <= wdata[15: 8];
	       end
	       vga_ready <= 1;
	       /* end else begin
		if (!vramren) begin
		vramren <= 1;
		vramraddr <= addr[7:0];
	       end else begin
		vga_rdata[31:0] <= {16'b0, vramrdata[15:0]};
		vga_ready <= 1;
	       end
	    end	*/
	    end else if (addr < 24'h800000) begin // FONT
	       if (wstrb[3:0] != 4'b0) begin
		  fontwen <= 1;
		  fontwaddr <= addr[13:2];
		  if (wstrb[0]) fontwdata[ 7: 0] <= wdata[ 7: 0];
		  if (wstrb[1]) fontwdata[15: 8] <= wdata[15: 8];
	       end
	       vga_ready <= 1;
	    end else begin
	       vga_ready <= 1;
	       case (addr)
		 24'h800000:
		   begin
		      vga_rdata <= {16'h0, cursx};
		      if (wstrb[0]) cursx[7:0]  <= wdata[ 7: 0];
		      if (wstrb[1]) cursx[15:8] <= wdata[15: 8];
		   end
		 24'h800004:
		   begin
		      vga_rdata <= {16'h0, cursy};
		      if (wstrb[0]) cursy[7:0]  <= wdata[ 7: 0];
		      if (wstrb[1]) cursy[15:8] <= wdata[15: 8];
		   end
		 24'h800008:
		   begin
		      vga_rdata <= {8'h0, curspal0};
		      if (wstrb[0]) curspal0[ 7: 0] <= wdata[ 7: 0];
		      if (wstrb[1]) curspal0[15: 8] <= wdata[15: 8];
		      if (wstrb[2]) curspal0[23:16] <= wdata[23:16];
		   end
		 24'h80000c:
		   begin
		      vga_rdata <= {8'h0, curspal1};
		      if (wstrb[0]) curspal1[ 7: 0] <= wdata[ 7: 0];
		      if (wstrb[1]) curspal1[15: 8] <= wdata[15: 8];
		      if (wstrb[2]) curspal1[23:16] <= wdata[23:16];
		   end
	       endcase
	    end
	 end
      end
   end

endmodule
