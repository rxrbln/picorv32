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
module dpram #(parameter integer WORDS = 1024,
	       parameter INITFILE = "charset32.hex",)
(
   input clk, wen, ren,
   input [15:0] waddr, raddr,
   input [31:0] wdata,
   input [3:0] wstrb,
   output reg [31:0] rdata,
);
   reg [31:0] mem [0:WORDS-1];

   // xxd -p < lena-1.raw | sed "s/\(..\)\(..\)/\2\1\n/g"
   initial $readmemh(INITFILE, mem, 0);

   always @(posedge clk) begin
      if (wen && wstrb[0]) mem[waddr][ 7: 0] <= wdata[ 7: 0];
      if (wen && wstrb[1]) mem[waddr][15: 8] <= wdata[15: 8];
      if (wen && wstrb[2]) mem[waddr][23:16] <= wdata[23:16];
      if (wen && wstrb[3]) mem[waddr][31:24] <= wdata[31:24];
      if (ren) rdata <= mem[raddr];
   end
endmodule

module vga(
   input clk, // system clk
   input resetn,
   
   input        sel,
   output       ready,
   input [ 3:0] wstrb,
   input [23:0] addr,
   input [31:0] wdata,
   output [31:0] rdata,

   output [3:0] gpdi_dp, gpdi_dn,
);

   wire   pixclk = clk; // same 25MHz

      ////////////////////////////
   // RGB output

   
   reg [7:4] vid_r;
   reg [7:4] vid_g;
   reg [7:4] vid_b;
   reg 	     vid_hs;
   reg 	     vid_vs;
   reg 	     vid_de;
   
   wire [23:0] color;
   assign color = {vid_r, 4'b0, vid_g, 4'b0, vid_b, 4'b0};
   
   // clock generator
   wire clk_250MHz, clk_125MHz, clk_25MHz;
   wire clk_locked;
   
    clk_25_250_125_25
    clock_instance
    (
      .clki(clk),
      .clko(clk_250MHz),
      .clks1(clk_125MHz),
      .clks2(clk_25MHz),
      .locked(clk_locked)
    );
   

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

    // VGA to digital video converter
    wire [1:0] tmds[3:0];
    vga2dvid vga2dvid_instance
    (
      .clk_pixel(clk_25MHz),
      .clk_shift(clk_125MHz),
      .in_color(color),
      .in_hsync(hsync),
      .in_vsync(vsync),
      .in_blank(!data_en),
      .out_clock(tmds[3]),
      .out_red(tmds[2]),
      .out_green(tmds[1]),
      .out_blue(tmds[0]),
      .resetn(clk_locked),
    );

    // output TMDS SDR/DDR data to fake differential lanes
    fake_differential fake_differential_instance
    (
      .clk_shift(clk_125MHz),
      .in_clock(tmds[3]),
      .in_red(tmds[2]),
      .in_green(tmds[1]),
      .in_blue(tmds[0]),
      .out_p(gpdi_dp),
      .out_n(gpdi_dn)
    );
   

   ////////////////////////////
   // mmio bus buffer interface
   reg [31:0] vga_rdata = 0;
   reg 	      vga_ready;
   assign ready = vga_ready;
   assign rdata = vga_rdata;
   

   
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
   wire [31:0] vramrdata;
   reg [31:0] vramwdata;
   reg [3:0] vramwstrb;
   reg [15:0] vramraddr;
   reg [15:0] vramwaddr;
   
   dpram #(.WORDS(32768/2),
	   .INITFILE("lena256.hex"),)
   vram (
      .clk(pixclk),
      .ren(vramren),
      .wen(vramwen),
      .raddr(vramraddr),
      .waddr(vramwaddr),
      .rdata(vramrdata),
      .wdata(vramwdata),
      .wstrb(vramwstrb),
   );

   reg 	fontwen = 0;
   reg 	fontren = 0;
   wire [31:0] fontrdata;
   reg [31:0] fontwdata;
   reg [3:0] fontwstrb;
   
   reg [10:0] fontwaddr;
   reg [10:0] fontraddr;
   
   dpram #(.INITFILE("charset32.hex"),) // "320240bw2.hex"
   fontram (
      .clk(pixclk),
      .ren(fontren),
      .wen(fontwen),
      .raddr(fontraddr),
      .waddr(fontwaddr),
      .rdata(fontrdata),
      .wdata(fontwdata),
      .wstrb(fontwstrb),
    );

   // text or graphic mode?
   reg [3:0] gfxmode = 0; // bit depth
   // TODO: dynamic bit-depth pre-load and shifting
   // wire [3:0] pxpreload;
   // assign pxpreload = 4'd31; // 32 >> (gfxmode)

   reg [31:0] pxdata;
   reg [7:0]  attr;
   reg [7:0]  row;
   
   wire [7:0] mask = 8'b10000000 >> xpos[2:0];

   // text mode, only load every 8 pixels

   reg [8:0]  col;
   wire [3:0]  fgcol;
   wire [3:0]  bgcol;
   wire        blink;
   assign fgcol = attr[3:0];
   assign bgcol = attr[6:4];
   assign blink = attr[7:7];

   reg [11:0]  fb_rgb;
   reg [11:0]  pal[256];
   initial pal[0] = 12'h000; // black
   initial pal[1] = 12'h00a; // blue
   initial pal[2] = 12'h0a0; // green
   initial pal[3] = 12'h0aa; // cyan
   initial pal[4] = 12'ha00; // red
   initial pal[5] = 12'ha0a; // magenta
   initial pal[6] = 12'haa0; // brown !!
   initial pal[7] = 12'haaa; // light gray
   initial pal[8] = 12'h555; // "light black" / dark gray
   initial pal[9] = 12'h55f; // light blue
   initial pal[10] = 12'h5f5; // light green
   initial pal[11] = 12'h5ff; // light cyan
   initial pal[12] = 12'hf55; // light red
   initial pal[13] = 12'hf5f; // light magenta
   initial pal[14] = 12'hff5; // yellow
   initial pal[15] = 12'hfff; // white
   initial $readmemh("vgapal.hex", pal, 0);

   
   //  16x16               // mask             color
   reg [31:0]  cursor[16]; // cursor planes
   initial cursor[0] = 32'b10000000000000000000000000000000;
   initial cursor[1]  = 32'b11000000000000000100000000000000;
   initial cursor[2]  = 32'b11100000000000000110000000000000;
   initial cursor[3]  = 32'b11110000000000000111000000000000;
   initial cursor[4]  = 32'b11111000000000000111100000000000;
   initial cursor[5]  = 32'b11111100000000000111110000000000;
   initial cursor[6]  = 32'b11111110000000000111111000000000;
   initial cursor[7]  = 32'b11111111100000000111111100000000;
   initial cursor[8]  = 32'b11111111110000000111111110000000;
   initial cursor[9]  = 32'b11111111111000000111111111000000;
   initial cursor[10] = 32'b11111111000000000111111000000000;
   initial cursor[11] = 32'b11100111100000000110011000000000;
   initial cursor[12] = 32'b10000011110000000000001100000000;
   initial cursor[13] = 32'b00000011110000000000001100000000;
   initial cursor[14] = 32'b00000001111000000000000110000000;
   initial cursor[15] = 32'b00000000110000000000000000000000;
   
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

   wire isread = wstrb[3:0] != 4'hf;
   wire iswrite = wstrb[3:0] != 4'h0;
   reg 	readready = 0;
   
   wire [15:0] textAddr = ((ypos & 16'hFFF0) << 3) | ((xpos[15:0] + 7) >> 3);
   wire textHiWord = textAddr & 1;
   
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
	 if (gfxmode == 3'b0) begin
	    // text mode: pre-load every 8 pixels
	    if (xpos[2:0] == 3) begin
	       // load char index from vram, * 256 bytes, 128 "words" per row
	       // interleaved text color attribute
	       vramraddr <= textAddr >> 1;
	       vramren <= 1;
	    end else if (xpos[2:0] == 5) begin
	       vramren <= 0;
	       fontraddr <= {textHiWord ? vramrdata[23:16] : vramrdata[7:0], ypos[3:2]};
	       fontren <= 1;
	    end else if (xpos[2:0] == 7) begin
	       // transfer pre-loaded at begin of each pixel
	       fontren <= 0;
	       case (ypos[1:0])
		 2'b00: row <= fontrdata[ 7: 0];
		 2'b01: row <= fontrdata[15: 8];
		 2'b10: row <= fontrdata[23:16];
		 2'b11: row <= fontrdata[31:24];
	       endcase
	       attr <= textHiWord ? vramrdata[31:24] : vramrdata[15:8];
	    end
	    
	    if (row & mask && (!blink || frame & 5'b10000)) begin
	       col = {0, fgcol};
	    end else begin
	       col = {bgcol};
	    end
	    
	 end else begin // if (gfxmode)
	    // graphic mode, "double scan" 320x240 => 640x480
	    if (xpos == -16 && ypos == 0)
	      ipos <= 0;
	    else if (xpos == -16 && ypos[0])
	      ipos <= ipos - 320; // "double scan" every 2nd line
	    else if (data_en)
	      ipos <= ipos + xpos[0]; // "double scan" every 2nd pixel
	    
	    // graphic mode: pre-load every N pixels
	    if (xpos[1:0] == 3) begin // TODO: gfxpreload
	       // generate RAM addres to load px from vram
	       if (!ipos[16]) begin // ipos[15:4] < 4096
		  vramraddr <= ipos[15:2]; // 8bpp -> / 16
		  vramren <= 1;
	       end else begin
		  fontraddr <= ipos[14:2]; // 1bpp -> / 16
		  fontren <= 1;
	       end
	    end else if (xpos[1:0] == 0) begin
	       // transfer pre-loaded at begin of each pixel
	       if (!ipos[16]) begin // ipos[15:4] < 4096
		  vramren <= 0;
		  pxdata = vramrdata;
	       end else begin
		  fontren <= 0;
		  pxdata = fontrdata;
	       end
	    end

	    // TODO: dynamic shift gfxmode
	    col = pxdata >> (8 * ipos[1:0]);
	 end
	 
	 fb_rgb = pal[col];

	 // cursor overlay
	 if (cursix && cursiy && cursix <= 16 && cursiy <= 16) begin
	    curs_row = cursor[cursiy];
	    
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
	       if (addr[17]) begin
		  //  // 2nd "bank" FONT
		  // MUX read access
		  if (isread && xpos[2:0] == 0) begin
		     fontren <= 1;
		     fontraddr <= addr[12:2];
		  end else if (isread && fontren && xpos[2:0] == 2) begin
		     // the riscv core runs half the clock
		     // w/o read buffer we get some glitches
		     fontren <= 0;
		     vga_rdata[31:0] <= fontrdata[31:0];
		     readready <= 1;
		     vga_ready <= !iswrite; // micro optimization
		  end else if (!isread || (isread && readready)) begin
		     fontwen <= iswrite;
		     fontwaddr <= addr[13:2];
		     fontwstrb <= wstrb;
		     fontwdata <= wdata;
		     vga_ready <= 1;
		  end
	       end else begin // 1st "bank" VRAM
		  // MUX read access
		  if (isread && xpos[2:0] == 0) begin
		     vramren <= 1;
		     vramraddr <= addr[16:2];
		  end else if (isread && vramren && xpos[2:0] == 2) begin
		     // the riscv core runs half the clock
		     // w/o read buffer we get some glitches
		     vramren <= 0;
		     vga_rdata[31:0] <= vramrdata[31:0];
		     readready <= 1;
		     vga_ready <= !iswrite; // micro optimization
		  end else if (!isread || (isread && readready)) begin
		     vramwen <= iswrite;
		     vramwaddr <= addr[16:2];
		     vramwstrb <= wstrb;
		     vramwdata <= wdata;
		     vga_ready <= 1;
		  end
	       end
	    end else begin // high bit set: CTRL regs
	       vga_ready <= 1;
	       case (addr)
		 24'h8ffffc: // control register
		   begin
		      vga_rdata[2:0] <= gfxmode;
		      if (wstrb[0]) gfxmode[3:0] <= wdata[3:0];
		   end
		 24'h800000: // hw cursor x/y
		   begin
		      vga_rdata[15:0] <= cursx;
		      if (wstrb[0]) cursx[ 7:0] <= wdata[ 7:0];
		      if (wstrb[1]) cursx[15:8] <= wdata[15:8];
		   end
		 24'h800004:
		   begin
		      vga_rdata[15:0] <= cursy;
		      if (wstrb[0]) cursy[ 7:0] <= wdata[ 7:0];
		      if (wstrb[1]) cursy[15:8] <= wdata[15:8];
		   end
		 24'h800008: // hw cursor palette
		   begin
		      //vga_rdata[23:0] <= curspal0;
		      if (wstrb[0]) curspal0[ 3:0] <= wdata[ 7: 4];
		      if (wstrb[1]) curspal0[ 7:4] <= wdata[15:12];
		      if (wstrb[2]) curspal0[11:8] <= wdata[23:20];
		   end
		 24'h80000c:
		   begin
		      //vga_rdata[23:0] <= curspal1;
		      if (wstrb[0]) curspal1[ 3:0] <= wdata[ 7: 4];
		      if (wstrb[1]) curspal1[ 7:4] <= wdata[15:12];
		      if (wstrb[2]) curspal1[11:8] <= wdata[23:20];
		   end
		 default:
		   begin
		      // TODO: reading like this might be a bit "expensive"
		      vga_rdata[19:0] <= {pal[addr[9:2]][3:0], 4'b0, pal[addr[9:2]][3:0], 4'b0, pal[addr[9:2]][3:0]};
		      if (wstrb != 4'b0) // palette write, convert 24 to 12 bit
			pal[addr[9:2]] <= {wdata[23:20], wdata[15:12], wdata[7:4]};
		   end
	       endcase
	    end
	 end
      end
   end

endmodule
