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
	input clk, clk_25mhz, clk_mem,
	output reg [7:0] led,
	output uart_tx,
       	input uart_rx,
	output [3:0] audio_l,
	output [3:0] audio_r,
	output [3:0] gpdi_dp, gpdi_dn,

	// SPI flash
	input flash_sck,
	output flash_csn,
	output flash_mosi,
	input  flash_miso,
	output flash_wpn,
	output flash_holdn,

	// sd memory
	output sdram_csn,       // chip select
	output sdram_clk,       // clock to SDRAM
	output sdram_cke,       // clock enable to SDRAM
	output sdram_rasn,      // SDRAM RAS
	output sdram_casn,      // SDRAM CAS
	output sdram_wen,       // SDRAM write-enable
	output [12:0] sdram_a,  // SDRAM address bus
	output [1:0] sdram_ba,  // SDRAM bank-address
	output [1:0] sdram_dqm, // byte select
	inout [15:0] sdram_d,   // data bus to/from SDRAM
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


        // sdram bi-directional output-enable
        wire [15:0] sd_data_in;
        wire [15:0] sd_data_out;
        TRELLIS_IO #(.DIR("BIDIR"))
        sdio_tristate[15:0] (
          .B(sdram_d),
          .I(sd_data_out),
          .O(sd_data_in),
          .T(!sdram_wen)
       );

   assign sdram_cke = 1'b1;
   assign sdram_clk = clk_mem;
   wire sdram_sel = iomem_valid && iomem_addr[29]; // 3rd MSB for a test
   wire [15:0] sdram_rdata;

   reg[3:0]  sdram_cycle = 0;
   wire sdram_write = sdram_sel && (mem_wstrb != 4'b0);
   wire sdram_read = sdram_sel && !sdram_write;
   
   sdram sdram(
    // physical interface
    .sd_data_in(sd_data_in),
    .sd_data_out(sd_data_out),
    .sd_addr(sdram_a),
    .sd_dqm(sdram_dqm),
    .sd_cs(sdram_csn),
    .sd_ba(sdram_ba),
    .sd_we(sdram_wen),
    .sd_ras(sdram_rasn),
    .sd_cas(sdram_casn),
    // system interface
    .clk(clk_mem),
    .clkref(clk),
    .init(!resetn),
    // cpu/chipset interface
    .addr(mem_addr | (sdram_cycle[3] ? 2 : 0)),
    .we(sdram_write),
    .dqm(sdram_cycle[3] ? mem_wstrb[3:2] : mem_wstrb[1:0]),
    .din(sdram_cycle[3] ? mem_wdata[31:16] : mem_wdata[15:0]),
    .oeA(sdram_read),
    .doutA(sdram_rdata),
  );


       // flash quad-SPI bi-directional output-enable
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
	   sdram_cycle <= 0;
	   
	   if (iomem_valid && mem_addr == 32'h 03000000) begin
	      if (iomem_wstrb[0]) led <= iomem_wdata[7:0];
	      iomem_ready <= 1'b1;
	      iomem_rdata <= led;
	   end else if (sdram_sel) begin
	      sdram_cycle <= sdram_cycle + 1;
	      if (sdram_read && sdram_cycle[2:0] == 3'b111) begin
		 if (sdram_cycle[3])
		   iomem_rdata[31:16] <= sdram_rdata;
		 else
		   iomem_rdata[15:0] <= sdram_rdata;
	      end
	      iomem_ready <= (sdram_cycle == 4'b1111);
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

// xxx MHz system memory clock
module pll(input clki, 
    output clks1,
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
        .CLKOP_CPHASE(3),
        .OUTDIVIDER_MUXA("DIVA"),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(7),
        .CLKOS_ENABLE("ENABLED"),
        .CLKOS_DIV(28),
        .CLKOS_CPHASE(3),
        .CLKOS_FPHASE(0),
        .CLKFB_DIV(24),
        .CLKI_DIV(7),
        .FEEDBK_PATH("INT_OP")
    ) pll_i (
        .CLKI(clki),
        .CLKFB(clkfb),
        .CLKINTFB(clkfb),
        .CLKOP(clkop),
        .CLKOS(clks1),
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

    // SPI flash
    output flash_csn,
    output flash_mosi,
    input flash_miso,
    output flash_wpn,
    output flash_holdn,

    // analog audio
    output [3:0] audio_l,
    output [3:0] audio_r,

    // not-HDMI
    output [3:0] gpdi_dp, gpdi_dn,

    // sdram
    output sdram_csn,       // chip select
    output sdram_clk,       // clock to SDRAM
    output sdram_cke,       // clock enable to SDRAM
    output sdram_rasn,      // SDRAM RAS
    output sdram_casn,      // SDRAM CAS
    output sdram_wen,       // SDRAM write-enable
    output [12:0] sdram_a,  // SDRAM address bus
    output [1:0] sdram_ba,  // SDRAM bank-address
    output [1:0] sdram_dqm, // byte select
    inout [15:0] sdram_d,   // data bus to/from SDRAM
);
   
   assign wifi_gpio0 = 1'b1;

   wire sysclk; // higher system clock
   wire clk;
   
   pll pll(
    .clki(clk_25mhz),
    .clko(sysclk),
    .clks1(clk),
   );
   
   // flash SPI clock
   wire flash_sck;
   wire tristate = 1'b0;
   USRMCLK u1 (.USRMCLKI(flash_sck), .USRMCLKTS(tristate));

attosoc soc(
    .clk(clk),
    .clk_mem(sysclk),
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
    .flash_mosi(flash_mosi),
    .flash_miso(flash_miso),
    .flash_holdn(flash_holdn),
    .flash_wpn(flash_wpn),

    .sdram_csn(sdram_csn),
    .sdram_clk(sdram_clk),
    .sdram_cke(sdram_cke),
    .sdram_rasn(sdram_rasn),
    .sdram_casn(sdram_casn),
    .sdram_wen(sdram_wen),
    .sdram_a(sdram_a),
    .sdram_ba(sdram_ba),
    .sdram_dqm(sdram_dqm),
    .sdram_d(sdram_d),
);

endmodule

