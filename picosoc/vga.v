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
   output reg [15:0] rdata,
);
   parameter INITFILE = "charset16.hex";
   reg [15:0] mem [0:4096-1];

   // xxd -p < lena-1.raw | sed "s/\(..\)\(..\)/\2\1\n/g"
   initial $readmemh(INITFILE, mem, 0);

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
   reg [31:0] vga_rdata = 0;
   reg 	      vga_ready;
   assign ready = vga_ready;
   assign rdata = vga_rdata;
   
   ////////////////////////////
   // video timing
   
   wire    vsync;
   wire    hsync;
   wire    data_en;

   wire signed [15:0] xpos;
   wire signed [15:0] ypos;
   
   
   video_timing video_timing_inst (
      .clk(pixclk),
      .hsync(hsync),
      .vsync(vsync),
      .data_en(data_en),
      .xpos(xpos),
      .ypos(ypos),
   );

   ////////////////////////////
   // RGB output
   
   reg [7:4] vid_r;
   reg [7:4] vid_g;
   reg [7:4] vid_b;
   reg 	     vid_hs;
   reg 	     vid_vs;
   reg 	     vid_de;
   
   reg [8:0]   frame = 0;
   reg [23:0]  ipos = 0; // linear FB counter
   
   reg 	       hsync_prev = 0;
   reg 	       vsync_prev = 0;
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
   
   dpram #(
      .INITFILE("320240bw.hex"),
   ) nvram (
      .clk(pixclk),
      .ren(vramren),
      .wen(vramwen),
      .raddr(vramraddr),
      .waddr(vramwaddr),
      .rdata(vramrdata),
      .wdata(vramwdata),
   );

   reg 	fontwen = 0;
   reg 	fontren = 0;
   wire [15:0] fontrdata;
   reg [15:0] fontwdata;
   reg [10:0] fontwaddr;
   reg [10:0] fontraddr;
   
   dpram #(
      .INITFILE("charset16.hex"), // "320240bw2.hex"
   ) fontram (
      .clk(pixclk),
      .ren(fontren),
      .wen(fontwen),
      .raddr(fontraddr),
      .waddr(fontwaddr),
      .rdata(fontrdata),
      .wdata(fontwdata),
    );

   // text or graphic mode?
   reg textmode = 1;

   reg [15:0] pxdata;
   reg [7:0]  attr;
   reg [7:0]  row;
   
   wire [7:0] mask;
   assign mask = 8'b10000000 >> xpos[2:0];

   // text mode, only load every 8 pixels

   reg [3:0]  col;
   wire [3:0]  fgcol;
   wire [3:0]  bgcol;
   wire        blink;
   assign fgcol = attr[3:0];
   assign bgcol = attr[6:4];
   assign blink = attr[7:7];

   reg [11:0]  fb_rgb;
   reg [11:0]  pal0 = 12'h000; // black
   reg [11:0]  pal1 = 12'h00a; // blue
   reg [11:0]  pal2 = 12'h0a0; // green
   reg [11:0]  pal3 = 12'h0aa; // cyan
   reg [11:0]  pal4 = 12'ha00; // red
   reg [11:0]  pal5 = 12'ha0a; // magenta
   reg [11:0]  pal6 = 12'haa0; // brown !!
   reg [11:0]  pal7 = 12'haaa; // light gray
   reg [11:0]  pal8 = 12'h555; // "light black" / dark gray
   reg [11:0]  pal9 = 12'h55f; // light blue
   reg [11:0]  pal10 = 12'h5f5; // light green
   reg [11:0]  pal11 = 12'h5ff; // light cyan
   reg [11:0]  pal12 = 12'hf55; // light red
   reg [11:0]  pal13 = 12'hf5f; // light magenta
   reg [11:0]  pal14 = 12'hff5; // yellow
   reg [11:0]  pal15 = 12'hfff; // white
   
   //  16x16               // mask             color
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

   wire [15:0] cursix;
   wire [15:0] cursiy;
   assign cursix = xpos + 16 - cursx; // 1 .. 16
   assign cursiy = ypos + 16 - cursy;
   
   reg [11:0]  curspal0 = 12'h000; // cursor palette
   reg [11:0]  curspal1 = 12'hfff;
   reg cursp0;
   reg cursp1;

   wire needsread = wstrb[3:0] != 4'hf;
   wire iswrite = wstrb[3:0] != 4'h0;
   reg 	readready = 0;
   
   // ----------------------------------------------------------------------------
   // Video output
   // ----------------------------------------------------------------------------
   
      always @(posedge pixclk) begin
      hsync_prev <= hsync;
      vsync_prev <= vsync;
      vsync_pulse <= vsync & ~vsync_prev;
      
      frame <= frame + vsync_pulse;
      
      // text & graphic pixel generation
	 // simply always generate signal, even if not active
      if (1) begin // !vsync && !hsync
	 if (textmode) begin
	    // text mode: pre-load every 8 pixels
	    if (xpos[2:0] == 3) begin
	       // load char index from vram, * 256 bytes, 128 "words" per row
	       // interleaved text color attribute
	       vramraddr <= ((ypos & 16'hFFF0) << 3) | ((xpos[15:0] + 7) >> 3);
	       vramren <= 1;
	    end else if (xpos[2:0] == 5) begin
	       vramren <= 0;
	       fontraddr <= {vramrdata[7:0], ypos[3:1]};
	       fontren <= 1;
	    end else if (xpos[2:0] == 7) begin
	       // transfer pre-loaded at begin of each pixel
	       fontren <= 0;
	       row <= ypos[0] ? fontrdata[15:8] : fontrdata[7:0];
	       attr <= vramrdata[15:8];
	    end
	    
	    if (row & mask && (!blink || frame & 5'b10000)) begin
	       col = fgcol;
	    end else begin
	       col = bgcol;
	    end
	    
	 end else begin // if (!textmode)
	    // graphic mode, "double scan" 320x240 => 640x480
	    if (xpos == -16 && ypos == 0)
	      ipos <= 0;
	    else if (xpos == -16 && ypos[0])
	      ipos <= ipos - 320; // "double scan" every 2nd line
	    else if (data_en)
	      ipos <= ipos + xpos[0]; // "double scan" every 2nd pixel
	    
	    // graphic mode: pre-load every 16 pixels
	    if (xpos[3:0] == 13) begin
	       // load px from vram
	       if (!ipos[16]) begin // ipos[15:4] < 4096
		  vramraddr <= ipos[15:4]; // 1bpp -> / 16
		  vramren <= 1;
	       end else begin
		  fontraddr <= ipos[14:4]; // 1bpp -> / 16
		  fontren <= 1;
	       end
	    end else if (xpos[3:0] == 15) begin
	       // transfer pre-loaded at begin of each pixel
	       if (!ipos[16]) begin // ipos[15:4] < 4096
		  vramren <= 0;
		  pxdata <= vramrdata[15:0];
	       end else begin
		  fontren <= 0;
		  pxdata <= fontrdata[15:0];
	       end
	    end
	    
	    if (pxdata & (16'h8000 >> ipos[3:0])) begin
	       col = 4'hf;
	    end else begin
	       col = 4'h0;
	    end
	 end
	 
	 case (col)
	   4'h1 : fb_rgb = pal1;
	   4'h2 : fb_rgb = pal2;
	   4'h3 : fb_rgb = pal3;
	   4'h4 : fb_rgb = pal4;
	   4'h5 : fb_rgb = pal5;
	   4'h6 : fb_rgb = pal6;
	   4'h7 : fb_rgb = pal7;
	   4'h8 : fb_rgb = pal8;
	   4'h9 : fb_rgb = pal9;
	   4'ha : fb_rgb = pal10;
	   4'hb : fb_rgb = pal11;
	   4'hc : fb_rgb = pal12;
	   4'hd : fb_rgb = pal13;
	   4'he : fb_rgb = pal14;
	   4'hf : fb_rgb = pal15;
	   default : fb_rgb = pal0;
	 endcase

	 // cursor overlay
	 if (cursix && cursiy && cursix <= 16 && cursiy <= 16) begin
	    case (cursiy)
	      4'h1 : curs_row = cursor0;
	      4'h2 : curs_row = cursor1;
	      4'h3 : curs_row = cursor2;
	      4'h4 : curs_row = cursor3;
	      4'h5 : curs_row = cursor4;
	      4'h6 : curs_row = cursor5;
	      4'h7 : curs_row = cursor6;
	      4'h8 : curs_row = cursor7;
	      4'h9 : curs_row = cursor8;
	      4'ha : curs_row = cursor9;
	      4'hb : curs_row = cursor10;
	      4'hc : curs_row = cursor11;
	      4'hd : curs_row = cursor12;
	      4'he : curs_row = cursor13;
	      4'hf : curs_row = cursor14;
	      default : curs_row = cursor15;
	    endcase // case (ypos - cursy)
	    
	    // transparent, invert, plane0, plane1?
	    cursp0 = curs_row[31:16] >> (16 - cursix);
	    cursp1 = curs_row[15:0] >> (16 - cursix);
	    if (cursp0) begin
	       if (cursp1)
		 {vid_r[7:4], vid_g[7:4], vid_b[7:4]} <= curspal1;
	       else
		 {vid_r[7:4], vid_g[7:4], vid_b[7:4]} <= curspal0;
	    end else begin
	       if (cursp1) // invert / "highlight"
		 {vid_r[7:4], vid_g[7:4], vid_b[7:4]} <= ~fb_rgb; //{8'hff - fb_rgb[23:16], 8'hff - fb_rgb[15:8], 8'hff - fb_rgb[7:0]}
	       else
		 {vid_r[7:4], vid_g[7:4], vid_b[7:4]} <= fb_rgb;
	    end
	 end else begin
	    {vid_r[7:4], vid_g[7:4], vid_b[7:4]} <= fb_rgb;
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

      // output 1 clk delayed in sync w/ our pixel generation!
      vid_hs <= hsync_prev;
      vid_vs <= vsync_prev;
      vid_de <= data_en;
      
      
      // mmio system bus interface
      if (resetn) begin
	 vga_ready <= 0;
	 vramwen <= 0;
	 fontwen <= 0;
	 
	 if (!sel) begin
	    readready <= 0;
	 end else begin
	    // must be aligned 16 bit writes, ..!
	    if (!addr[23]) begin // VRAM, not CTRL registers
	       if (addr[14]) begin
		  //  // 2nd "bank" FONT
		  // MUX read access
		  if (needsread && xpos[2:0] == 0) begin
		     fontren <= 1;
		     fontraddr <= addr[12:2];
		  end else if (needsread && fontren && xpos[2:0] == 2) begin
		     // the riscv core runs half the clock
		     // w/o read buffer we get some glitches
		     fontren <= 0;
		     vga_rdata[15:0] <= fontrdata[15:0];
		     readready <= 1;
		     vga_ready <= !iswrite; // micro optimization
		  end else if (!needsread || (needsread && readready)) begin
		     fontwen <= iswrite;
		     fontwaddr <= addr[13:2];
		     fontwdata[ 7: 0] <= wstrb[0] ? wdata[ 7: 0] : fontrdata[ 7:0];
		     fontwdata[15: 8] <= wstrb[1] ? wdata[15: 8] : fontrdata[15:8];
		     vga_ready <= 1;
		  end
	       end else begin // 1st "bank" VRAM
		  // MUX read access
		  if (needsread && xpos[2:0] == 0) begin
		     vramren <= 1;
		     vramraddr <= addr[13:2];
		  end else if (needsread && vramren && xpos[2:0] == 2) begin
		     // the riscv core runs half the clock
		     // w/o read buffer we get some glitches
		     vramren <= 0;
		     vga_rdata[15:0] <= vramrdata[15:0];
		     readready <= 1;
		     vga_ready <= !iswrite; // micro optimization
		  end else if (!needsread || (needsread && readready)) begin
		     vramwen <= iswrite;
		     vramwaddr <= addr[13:2];
		     vramwdata[ 7: 0] <= wstrb[0] ? wdata[ 7: 0] : vramrdata[ 7:0];
		     vramwdata[15: 8] <= wstrb[1] ? wdata[15: 8] : vramrdata[15:8];
		     vga_ready <= 1;
		  end
	       end
	    end else begin // high bit set: CTRL regs
	       vga_ready <= 1;
	       case (addr)
		 24'h8ffffc: // control register
		   begin
		      vga_rdata <= {23'h0, textmode};
		      if (wstrb[0]) textmode <= wdata[0];
		   end
		 24'h800000: // hw cursor x/y
		   begin
		      //vga_rdata <= {16'h0, cursx};
		      if (wstrb[0]) cursx[ 7:0] <= wdata[ 7:0];
		      if (wstrb[1]) cursx[15:8] <= wdata[15:8];
		   end
		 24'h800004:
		   begin
		      //vga_rdata <= {16'h0, cursy};
		      if (wstrb[0]) cursy[ 7:0] <= wdata[ 7:0];
		      if (wstrb[1]) cursy[15:8] <= wdata[15:8];
		   end
		 24'h800008: // hw cursor palette
		   begin
		      //vga_rdata <= {8'h0, curspal0};
		      if (wstrb[0]) curspal0[ 3:0] <= wdata[ 7: 4];
		      if (wstrb[1]) curspal0[ 7:4] <= wdata[15:12];
		      if (wstrb[2]) curspal0[11:8] <= wdata[23:20];
		   end
		 24'h80000c:
		   begin
		      //vga_rdata <= {8'h0, curspal1};
		      if (wstrb[0]) curspal1[ 3:0] <= wdata[ 7: 4];
		      if (wstrb[1]) curspal1[ 7:4] <= wdata[15:12];
		      if (wstrb[2]) curspal1[11:8] <= wdata[23:20];
		   end
	       endcase
	    end
	 end
      end
   end

endmodule
