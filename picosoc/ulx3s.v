/*
 *  ECP5 PicoRV32 demo
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *  Copyright (C) 2018  David Shah <dave@ds0.me>
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

`ifdef PICORV32_V
`error "attosoc.v must be read before picorv32.v!"
`endif

`define PICORV32_REGS picosoc_regs

module attosoc (
	input clk, clk_25mhz,
	output reg [7:0] led,
	output uart_tx,
       	input uart_rx,
	output [3:0] audio_l,
	output [3:0] audio_r,
	output [3:0] gpdi_dp, gpdi_dn,

	input flash_sck,
	output flash_csn,
	output flash_io0_oe,
	output flash_io1_oe,
	output flash_io2_oe,
	output flash_io3_oe,
	output flash_io0_do,
	output flash_io1_do,
	output flash_io2_do,
	output flash_io3_do,
	input  flash_io0_di,
	input  flash_io1_di,
	input  flash_io2_di,
	input  flash_io3_di,
);
   	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;

	always @(posedge clk) begin
		reset_cnt <= reset_cnt + !resetn;
	end

	parameter integer MEM_WORDS = 32768;
	parameter [31:0] STACKADDR = 32'h 0000_0000 + (4*MEM_WORDS);       // end of memory
	parameter [31:0] PROGADDR_RESET = 32'h 0020_0000; // 2 MB into flash

	reg [31:0] ram [0:MEM_WORDS-1];
	//initial $readmemh("ulx3s_fw.hex", ram);
        reg [31:0] 	   ram_rdata;
	reg ram_ready;

	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [31:0] mem_rdata;

   	wire dac_sel = iomem_valid && iomem_addr[30]; // 2nd MSB
        wire dac_ready;
   
	always @(posedge clk)
          begin
	     ram_ready <= mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS;
		if (mem_addr[31:24] == 8'h00 && mem_valid) begin
			if (mem_wstrb[0]) ram[mem_addr[23:2]][7:0] <= mem_wdata[7:0];
			if (mem_wstrb[1]) ram[mem_addr[23:2]][15:8] <= mem_wdata[15:8];
			if (mem_wstrb[2]) ram[mem_addr[23:2]][23:16] <= mem_wdata[23:16];
			if (mem_wstrb[3]) ram[mem_addr[23:2]][31:24] <= mem_wdata[31:24];
			ram_rdata <= ram[mem_addr[23:2]];
		end
        end

 
	wire iomem_valid;
	reg iomem_ready;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	wire [3:0] iomem_wstrb;
	reg [31:0] iomem_rdata;

	assign iomem_valid = mem_valid && (mem_addr[31:24] > 8'h 01);
	assign iomem_wstrb = mem_wstrb;
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;

        wire       spimem_ready;
        wire [31:0] spimem_rdata;

        /*
        flashmem flash_i (
           .clk(clk),
           .reset(!resetn),
           .valid(mem_valid && mem_addr >= 4*MEM_WORDS && mem_addr < 32'h 0200_0000),
           .addr(mem_addr[23:0]),
           .ready(spimem_ready),
           .rdata(spimem_rdata),
	   
           .spi_cs(flash_csn),
           .spi_sclk(flash_sck),
           .spi_mosi(flash_mosi),
           .spi_miso(flash_miso),
        );*/

   	spimemio spimemio (
		.clk    (clk),
		.resetn (resetn),
		.valid  (mem_valid && mem_addr >= 4*MEM_WORDS && mem_addr < 32'h 0200_0000),
		.ready  (spimem_ready),
		.addr   (mem_addr[23:0]),
		.rdata  (spimem_rdata),

		.flash_csb    (flash_csn   ),
		.flash_clk    (flash_sck   ),

		.flash_io0_oe (flash_io0_oe),
		.flash_io1_oe (flash_io1_oe),
		.flash_io2_oe (flash_io2_oe),
		.flash_io3_oe (flash_io3_oe),

		.flash_io0_do (flash_io0_do),
		.flash_io1_do (flash_io1_do),
		.flash_io2_do (flash_io2_do),
		.flash_io3_do (flash_io3_do),

		.flash_io0_di (flash_io0_di),
		.flash_io1_di (flash_io1_di),
		.flash_io2_di (flash_io2_di),
		.flash_io3_di (flash_io3_di),

		.cfgreg_we(spimemio_cfgreg_sel ? mem_wstrb : 4'b 0000),
		.cfgreg_di(mem_wdata),
		.cfgreg_do(spimemio_cfgreg_do)
	);

	wire spimemio_cfgreg_sel = mem_valid && (mem_addr == 32'h 0200_0000);
	wire [31:0] spimemio_cfgreg_do;

	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire simpleuart_reg_dat_wait;

	always @(posedge clk) begin
	   iomem_ready <= 1'b0;
	   if (iomem_valid && mem_addr == 32'h 03000000) begin
	      if (iomem_wstrb[0]) led <= iomem_wdata[7:0];
	      iomem_ready <= 1'b1;
	      iomem_rdata <= led;
	   end else if (vgamem_sel) begin
	      iomem_ready <= vgamem_ready;
	      iomem_rdata <= vgamem_rdata;
	   end else if (dac_sel) begin
	      iomem_ready <= dac_ready;
	   end
	end


	assign mem_ready = (iomem_valid && iomem_ready) ||
			   spimem_ready || spimemio_cfgreg_sel ||
	                   simpleuart_reg_div_sel ||
			   (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait) ||
			   ram_ready;

	assign mem_rdata = (iomem_valid && iomem_ready) ? iomem_rdata :
			   simpleuart_reg_div_sel ? simpleuart_reg_div_do :
			   simpleuart_reg_dat_sel ? simpleuart_reg_dat_do :
			   spimem_ready ? spimem_rdata :
			   spimemio_cfgreg_sel ? spimemio_cfgreg_do :
 			   ram_rdata;

	picorv32 #(
		.STACKADDR(STACKADDR),
		.PROGADDR_RESET(PROGADDR_RESET),
		.PROGADDR_IRQ(32'h 0000_0000),
		.BARREL_SHIFTER(1),
		.COMPRESSED_ISA(1),
		.ENABLE_MUL(1),
		.ENABLE_DIV(1),
		.ENABLE_IRQ(0),
		.ENABLE_IRQ_QREGS(0)
	) cpu (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  ),
	);


	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (uart_tx     ),
		.ser_rx      (uart_rx     ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait),
			       );
   
   wire dsd, dsd2;
   assign audio_l = {dsd, 3'b0};
   assign audio_r = {dsd2, 3'b0};

        audio audio (.clk(clk),
		     .dsd(dsd), .dsd2(dsd2),
		     .resetn(resetn),
		     .sel    (dac_sel),
		     .ready  (dac_ready),
		     .addr   (iomem_addr[23:0]),
		     .wstrb  (iomem_wstrb),
		     .wdata  (iomem_wdata),
		     );

      	reg vgamem_ready;
   	wire vgamem_sel = iomem_valid && iomem_addr[31]; // high bit set
	reg [31:0] vgamem_rdata;

        // vga core
        vga vga (
		 .clk(clk_25mhz),
		 .resetn(resetn),

		 .gpdi_dp(gpdi_dp),
		 .gpdi_dn(gpdi_dn),
		 
		.sel    (vgamem_sel),
		.ready  (vgamem_ready),
		.wstrb  (iomem_wstrb),
		.addr   (iomem_addr[23:0]),
		.wdata  (iomem_wdata),
		.rdata  (vgamem_rdata),
	);


endmodule

// Implementation note:
// Replace the following two modules with wrappers for your SRAM cells.

module picosoc_regs (
	input clk, wen,
	input [5:0] waddr,
	input [5:0] raddr1,
	input [5:0] raddr2,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2,
);
	reg [31:0] regs [0:31];

	always @(posedge clk)
		if (wen) regs[waddr[4:0]] <= wdata;

	assign rdata1 = regs[raddr1[4:0]];
	assign rdata2 = regs[raddr2[4:0]];
endmodule

 // 40 MHz
module pll(input clki,
    output locked,
    output clko
);
wire clkfb;
wire clkos;
wire clkop;
(* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
EHXPLLL #(
        .PLLRST_ENA("DISABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .CLKOP_FPHASE(0),
        .CLKOP_CPHASE(7),
        .OUTDIVIDER_MUXA("DIVA"),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(15),
        .CLKFB_DIV(8),
        .CLKI_DIV(5),
        .FEEDBK_PATH("INT_OP")
    ) pll_i (
        .CLKI(clki),
        .CLKFB(clkfb),
        .CLKINTFB(clkfb),
        .CLKOP(clkop),
        .RST(1'b0),
        .STDBY(1'b0),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b0),
        .PHASESTEP(1'b0),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
        .LOCK(locked)
	);
assign clko = clkop;
endmodule

module ulx3s(
    input clk_25mhz,
    output [7:0] led,
    output ftdi_rxd,
    input ftdi_txd,
    output wifi_gpio0,

    output flash_csn,
    output flash_mosi,
    input flash_miso,
    output flash_wpn,
    output flash_holdn,

    output [3:0] audio_l,
    output [3:0] audio_r,
    
    output [3:0] gpdi_dp, gpdi_dn,
);
   
   assign wifi_gpio0 = 1'b1;

wire clk;
pll pll(
    .clki(clk_25mhz),
    .clko(clk)
);


  wire flash_sck;
  wire tristate = 1'b0;
  USRMCLK u1 (.USRMCLKI(flash_sck), .USRMCLKTS(tristate));


   wire flash_io0_oe, flash_io0_do, flash_io0_di;
   wire flash_io1_oe, flash_io1_do, flash_io1_di;
   wire flash_io2_oe, flash_io2_do, flash_io2_di;
   wire flash_io3_oe, flash_io3_do, flash_io3_di;

   TRELLIS_IO #(.DIR("BIDIR"))
   flash_io_buffer0 [3:0] (
      .B({flash_mosi, flash_miso, flash_wpn, flash_holdn}),
      .I({flash_io0_do, flash_io1_do, flash_io2_do, flash_io3_do}),
      .O({flash_io0_di, flash_io1_di, flash_io2_di, flash_io3_di}),
      .T({!flash_io0_oe, !flash_io1_oe, !flash_io2_oe, !flash_io3_oe}));
   
attosoc soc(
    .clk(clk),
    .clk_25mhz(clk_25mhz),
    .led(led),
    
    .uart_tx(ftdi_rxd),
    .uart_rx(ftdi_txd),
    
    .audio_l(audio_l),
    .audio_r(audio_r),
    
    .gpdi_dp(gpdi_dp),
    .gpdi_dn(gpdi_dn),

    .flash_csn(flash_csn),
    .flash_sck(flash_sck),
//    .flash_mosi(flash_mosi),
//    .flash_miso(flash_miso),
    .flash_io0_oe (flash_io0_oe),
    .flash_io1_oe (flash_io1_oe),
    .flash_io2_oe (flash_io2_oe),
    .flash_io3_oe (flash_io3_oe),
    
    .flash_io0_do (flash_io0_do),
    .flash_io1_do (flash_io1_do),
    .flash_io2_do (flash_io2_do),
    .flash_io3_do (flash_io3_do),

    .flash_io0_di (flash_io0_di),
    .flash_io1_di (flash_io1_di),
    .flash_io2_di (flash_io2_di),
    .flash_io3_di (flash_io3_di),
);
endmodule

