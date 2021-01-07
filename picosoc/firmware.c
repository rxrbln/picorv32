/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *  Copyright (C) 2020  René Rebe <rene@exactcode.de>
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

/* TODO:
 * malloc / free
 * stdio
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "utility/C.h" // ARRAY_SIZE

typedef long size_t;
extern "C" {
#include "libc/strlen.c"
#include "libc/strncmp.c"
#include "libc/strncasecmp.c"
#include "libc/memset.c"
#include "libc/ctype.h"
#include "libc/sprintf.c"
#include "libc/sys/param.h" // MIN, MAX
}

#include "Endianess.hh"

#if 1
#define RDCYCLE(x) asm volatile ("rdcycle %0" : "=r"(x));
#define RDINSTR(x) asm volatile ("rdinstret %0" : "=r"(x));
#else
#define RDCYCLE(x) asm volatile ("csrrs %0, 0xb00, zero" : "=r"(x));
#define RDINSTR(x) asm volatile ("csrrs %0, 0xb02, zero" : "=r"(x));
#endif

using Exact::EndianessConverter;
using Exact::LittleEndianTraits;
using Exact::BigEndianTraits;

#ifdef ICEBREAKER
#  define MEM_TOTAL 0x20000 /* 128 KB */
#elif HX8KDEMO
#  define MEM_TOTAL 0x200 /* 2 KB */
#elif ULX3S
#  define MEM_TOTAL 0x20000 /* 128 KB */
#else
#  error "Set -DICEBREAKER, -DHX8KDEMO or -DULX3S when compiling firmware.c"
#endif

const uint32_t SYSCLK =
#ifdef ULX3S
  41666667;
#else
  12937000;
#endif

// a pointer to this is a null pointer, but the compiler does not
// know that because "sram" is a linker symbol from sections.lds.
extern uint32_t sram;

#define reg_spictrl (*(volatile uint32_t*)0x02000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)

#define reg_leds (*(volatile uint32_t*)0x03000000)
#define reg_uart_midi (*(volatile uint8_t*)0x03000010)

#define reg_ps2 (*(volatile uint32_t*)0x03000020)

#define reg_dac ((volatile uint32_t*)0x40000000)
#define reg_fm ((volatile uint32_t*)0x40000010)
#define reg_fm16 ((volatile uint16_t*)0x40000010)

static uint16_t* vga_vram = (uint16_t*)0x80000000;
static uint32_t* vga_vram32 = (uint32_t*)0x80000000;
static uint16_t* vga_font = (uint16_t*)0x80002000;

static uint32_t* sdram = (uint32_t*)0x20000000;

// --------------------------------------------------------

extern uint32_t flashio_worker_begin;
extern uint32_t flashio_worker_end;

static uint32_t vgax = 0, vgay = 0;

void flashio(uint8_t *data, int len, uint8_t wrencmd)
{
	uint32_t func[&flashio_worker_end - &flashio_worker_begin];

	uint32_t *src_ptr = &flashio_worker_begin;
	uint32_t *dst_ptr = func;

	while (src_ptr != &flashio_worker_end)
		*(dst_ptr++) = *(src_ptr++);

	((void(*)(uint8_t*, uint32_t, uint32_t))func)(data, len, wrencmd);
}

#ifdef HX8KDEMO
void set_flash_qspi_flag()
{
	uint8_t buffer[8];
	uint32_t addr_cr1v = 0x800002;

	// Read Any Register (RDAR 65h)
	buffer[0] = 0x65;
	buffer[1] = addr_cr1v >> 16;
	buffer[2] = addr_cr1v >> 8;
	buffer[3] = addr_cr1v;
	buffer[4] = 0; // dummy
	buffer[5] = 0; // rdata
	flashio(buffer, 6, 0);
	uint8_t cr1v = buffer[5];

	// Write Enable (WREN 06h) + Write Any Register (WRAR 71h)
	buffer[0] = 0x71;
	buffer[1] = addr_cr1v >> 16;
	buffer[2] = addr_cr1v >> 8;
	buffer[3] = addr_cr1v;
	buffer[4] = cr1v | 2; // Enable QSPI
	flashio(buffer, 5, 0x06);
}

void set_flash_latency(uint8_t value)
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | ((value & 15) << 16);

	uint32_t addr = 0x800004;
	uint8_t buffer_wr[5] = {0x71,
				(uint8_t)(addr >> 16), (uint8_t)(addr >> 8),
				(uint8_t)(addr), 0x70 | value};
	flashio(buffer_wr, 5, 0x06);
}

void set_flash_mode_spi()
{
	reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00000000;
}

void set_flash_mode_dual()
{
	reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00400000;
}

void set_flash_mode_quad()
{
	reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00200000;
}

void set_flash_mode_qddr()
{
	reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00600000;
}
#endif

#if defined(ICEBREAKER) || defined(ULX3S)
void set_flash_qspi_flag()
{
	uint8_t buffer[8];

	// Read Configuration Registers (RDCR1 35h)
	buffer[0] = 0x35;
	buffer[1] = 0x00; // rdata
	flashio(buffer, 2, 0);
	uint8_t sr2 = buffer[1];

	// Write Enable Volatile (50h) + Write Status Register 2 (31h)
	buffer[0] = 0x31;
	buffer[1] = sr2 | 2; // Enable QSPI
	flashio(buffer, 2, 0x50);
}

void set_flash_mode_spi()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00000000;
}

void set_flash_mode_dual()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00400000;
}

void set_flash_mode_quad()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00240000;
}

void set_flash_mode_qddr()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00670000;
}

void enable_flash_crm()
{
	reg_spictrl |= 0x00100000;
}
#endif

// --------------------------------------------------------

int open(const char* pathname, int flags = 0);
size_t read(int fd, void* buf, size_t cound);
int close(int fd);


uint8_t scroll = 0;

void do_scroll() {
  ++vgay;
  vgax = 0;
  if (vgay >= 30){
    if (!scroll) {
      vgay = 0; // simply wrap
    } else {
      // scroll, only visible area, 32-bit (2 char + attr) at a time
      for (vgay = 0; vgay < 30 - scroll; ++vgay)
	for (int x = 0; x < 80/2; ++x)
	  vga_vram32[vgay * 128/2 + x] = vga_vram32[(vgay + scroll) * 128/2 + x];
      
      // clear lines after current, only for scroll > 1
      for (int y = vgay + 1; y < 30; ++y)
	for (int x = 0; x < 80/2; ++x)
	  vga_vram32[y * 128/2 + x] = 0;
    }
  }
  
  // clear last, new line
  for (int x = 0; x < 80/2; ++x)
    vga_vram32[vgay * 128/2 + x] = 0;
}

void putchar(char c)
{
  if (c == '\n')
    reg_uart_data = '\r';
 
#if 1
  vga_vram[0] = 0x6200 | 'R';
  vga_vram[1] = 0x1f00 | 'X';
  vga_vram[2] = 0x2900 | '3';
  vga_vram[3] = 0x8500 | '\xfd';

  if (c != '\n' && c != '\r') {
    if (vgax == 80) do_scroll();
    
    vga_vram[vgay * 128 + vgax++] =
      //(c == 'E' ? 0xe200 : 0x700)
      //(vgax + 1) << 8
      0x700 // color attribute: white on black
      | c;
  }
  if (c == '\n') {
    do_scroll();
  }
#endif
  reg_uart_data = c;
}

void print(const char *p)
{
	while (*p)
		putchar(*(p++));
}

// 1st and early for debug prints, ...
int printf(const char* format, ...)
  __attribute__ ((format (printf, 1, 2)));


extern "C"
uint32_t __bswapsi2 (uint32_t x) {
  return
    (x << 24) |
    ((x <<  8) & 0x00ff0000) |
    ((x >>  8) & 0x0000ff00) |
    (x >> 24);
}

#ifndef __riscv_muldiv

extern "C" {

struct DWstruct {
	int low, high;
};

typedef union {
	struct DWstruct s;
	long long ll;
} DWunion;

long long __ashldi3(long long u, int b)
{
	DWunion uu, w;
	int bm;
	
	if (b == 0)
		return u;

	uu.ll = u;
	bm = 32 - b;

	if (bm <= 0) {
		w.s.low = 0;
		w.s.high = (unsigned int) uu.s.low << -bm;
	} else {
		const unsigned int carries = (unsigned int) uu.s.low >> bm;

		w.s.low = (unsigned int) uu.s.low << b;
		w.s.high = ((unsigned int) uu.s.high << b) | carries;
	}

	return w.ll;
}


unsigned __udivsi3(unsigned dividend, unsigned divisor)
{
  static const int W = 24; // maximum nuber of bits in the dividend & divisor

  int64_t R = dividend; // partial remainder -- need 2*W bits
  uint32_t Q = 0; /* partial quotient */
  for (int8_t i = W; i >= 0; --i) {
    if (R >= 0) {
      R -= (uint64_t)divisor << i;
      Q += 1 << i;
    } else {
      R += (uint64_t)divisor << i;
      Q -= 1 << i;
    }
  }
  if (R < 0) {
    R += divisor;
    Q -= 1;
  }
  return Q;
}
unsigned __divsi3(unsigned, unsigned) __attribute__ ((weak, alias ("__udivsi3")));


unsigned __umodsi3(unsigned dividend, unsigned divisor)
{
  static const int W = 23; /* maximum nuber of bits in the dividend & divisor */

  int64_t R = dividend; /* partial remainder -- need 2*W bits */
  uint32_t Q = 0; /* partial quotient */
  for (int8_t i = W; i >= 0; --i) {
    if (R >= 0) {
      R -= (uint64_t)divisor << i;
      Q += 1 << i;
    } else {
      R += (uint64_t)divisor << i;
      Q -= 1 << i;
    }
  }
  if (R < 0) {
    R += divisor;
    Q -= 1;
  }
  return R;
}
unsigned __modsi3(unsigned, unsigned) __attribute__ ((weak, alias ("__umodsi3")));

unsigned __umulsi3(unsigned a, unsigned b)
{
  return a;
  uint32_t res = 0;
  while (b) {
    res += (b & 1) ? a : 0;
    a <<= 1;
    b >>= 1;
  }
  return res;
}
unsigned __mulsi3(unsigned, unsigned) __attribute__ ((weak, alias ("__umulsi3")));

} // extern "C"

#endif


int printf(const char* format, ...)
{
  char text[120]; // TODO: dynamic!
  
  va_list argp, argp2;
  va_start(argp, format);
  va_copy(argp2, argp);
  int ret = vsprintf(text, format, argp2);
  va_end(argp2);
  
  print(text);
  return ret;
}

const uint8_t ps2_set2[] = {
// 0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f
   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,'\t', '~',   0, //  0
   0,   0,   0,   0,   0, 'q', '1',   0,   0,   0, 'z', 's', 'a', 'w', '2',   0, // 10
   0, 'c', 'x', 'd', 'e', '4', '3',   0,   0, ' ', 'v', 'f', 't', 'r', '5',   0, // 20
 'n', 'n', 'b', 'h', 'g', 'y', '6',   0,   0,   0, 'm', 'j', 'u', '7', '8',   0, // 30
   0, '<', 'k', 'i', 'o', '0', '9',   0,   0, '>', '?', 'l', ':', 'p', '-',   0, // 40
   0,   0, '"',   0, '[', '+',   0,   0,   0,   0,'\r', ']',   0, '|',   0,   0, // 50
   0,   0,   0,   0,   0,   0,'\b',   0,   0,   0,   0,   0,   0,   0,   0,   0, // 60
   0,   0,   0,   0,   0,   0,'\e',   0,   0,   0,   0,   0,   0,   0,   0,   0, // 70
};

uint8_t ps2_release = false;
uint8_t ps2_ext = false;
uint8_t ps2_mod_shift = false;
uint8_t ps2_mod_ctrl = false;
uint8_t ps2_mod_alt = false;

char getchar_prompt(const char *prompt)
{
  int32_t c = -1;
  uint32_t cycles_begin, cycles_now, cycles;
  RDCYCLE(cycles_begin);
  
  reg_leds = ~0;
  
  if (prompt)
    print(prompt);

  while (c == -1) {
    RDCYCLE(cycles_now);
    cycles = cycles_now - cycles_begin;
    if (cycles > SYSCLK / 2) {
      if (prompt)
	print(prompt);
      cycles_begin = cycles_now;
      reg_leds = ~reg_leds;
    }
    
    // serial
    c = reg_uart_data;
    
    // ps2
    if (c == -1) {
      c = reg_ps2;
      if (c != -1) {
	//printf("%02x %d%d%d\n", c, ps2_mod_shift, ps2_mod_ctrl, ps2_mod_alt);
	switch (c) {
	case 0xf0: ps2_release = true; c = -1; break;
	case 0xe0: ps2_ext = true; c = -1; break;
	default:
	  if (ps2_ext)
	    c |= 0xe000;
	  
	  if (c == 0x11 || c == 0xe011) ps2_mod_alt = !ps2_release;
	  if (c == 0x12 || c == 0x59) ps2_mod_shift = !ps2_release;
	  if (c == 0x14 || c == 0xe014) ps2_mod_ctrl = !ps2_release;
	  
	  // so we got scancodes, map to ASCII
	  c = (c > 0 && c < ARRAY_SIZE(ps2_set2)) ?
	    ps2_set2[c] : -1;

	  if (ps2_mod_shift)
	    c = toupper(c);
	  
	  if (ps2_release)
	    c = -1; // ignore release for now
	  
	  ps2_release = ps2_ext = false;
	}
      }
    }
  }
  
  reg_leds = 0;
  return c;
}

char getchar()
{
	return getchar_prompt(0);
}

void getline(char* str, int len)
{
  int i;
  for (i = 0; i < len - 1; ++i) {
    str[i] = getchar_prompt(0);
    if (str[i] == '\n' || str[i] == '\r')
      break;
    printf("%c", str[i]); // echo
  }
  str[i] = 0;
}

void cmd_print_spi_state()
{
	print("SPI State:\n");
	printf("  LATENCY %d\n", (reg_spictrl >> 16) & 15);
	printf("  DDR %s\n", (reg_spictrl & (1 << 22)) != 0 ? "ON" : "OFF");
	printf("  QSPI %s\n", (reg_spictrl & (1 << 21)) != 0 ? "ON" : "OFF");
	printf("  CRM %s\n",  (reg_spictrl & (1 << 20)) != 0 ? "ON" : "OFF");
}

uint32_t xorshift32(uint32_t *state)
{
	/* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;

	return x;
}

void cmd_memtest(int ext)
{
	int cyc_count = 5;
	int stride = 256;
	uint32_t state;
	uint32_t errors;

	volatile uint32_t* base_word = (volatile uint32_t *) (ext ? sdram : 0);
	volatile uint8_t* base_byte = (volatile uint8_t *) (ext ? sdram : 0);
	const int mem_total = ext ? 64*1024*1024 : MEM_TOTAL;
	print("Running memtest\n");
	
#if 0
	if (ext) {
	  base_word[0] = 0x00112233;
	  //base_word[1] = 0x44556677;
	  //base_word[2] = 0x8899aabb;
	  //base_byte[3] = 0x67;
	  //base_byte[1] = 0x23;
	  //base_byte[2] = 0x45;
	  //base_byte[0] = 0x01;
	  printf("%08x %02x %02x %02x %02x\n", base_word[0],
		 base_byte[3], base_byte[2], base_byte[1], base_byte[0]);
	  printf("%08x %02x %02x %02x %02x\n", base_word[1],
		 base_byte[7], base_byte[6], base_byte[5], base_byte[4]);
	  printf("%08x %02x %02x %02x %02x\n", base_word[2],
		 base_byte[11], base_byte[10], base_byte[9], base_byte[8]);
	  return;
	}
#endif
	
	// Walk in all w/ 3 increments, word access
	if (ext)
	for (int i = 1; i <= 1; i++) {
		state = i;

		for (int word = 0; word < mem_total / sizeof(int); word += 3) {
			*(base_word + word) = word;
		}

		for (int word = 0; word < mem_total / sizeof(int); word += 3) {
		          uint32_t v = *(base_word + word), v2 = word;
			  if (v != v2) {
			    printf(" ***FAILED INDEX*** at %x: %x != %x\n", 4*word, v, v2);
			  ++errors;
			  return;
			}
		}

		print(".");
	}

	// Walk in stride increments, word access
	for (int i = 1; i <= cyc_count; i++) {
		state = i;

		for (int word = 0; word < mem_total / sizeof(int); word += stride) {
			*(base_word + word) = xorshift32(&state);
		}

		state = i;

		for (int word = 0; word < mem_total / sizeof(int); word += stride) {
		        uint32_t v = *(base_word + word), v2 = xorshift32(&state);
			if (v != v2) {
			        printf(" ***FAILED WORD*** at %x: %x != %x\n", 4*word, v, v2);
				++errors;
				//return;
			}
		}

		print(".");
	}

	// Byte access
	for (int byte = 0; byte < 128; byte++) {
		*(base_byte + byte) = (uint8_t) byte;
	}

	for (int byte = 0; byte < 128; byte++) {
		if (*(base_byte + byte) != (uint8_t) byte) {
		        printf(" ***FAILED BYTE*** at %x\n", byte);
			++errors;
			//return;
		}
	}
	
	if (!errors)
	  print(" passed\n");
}

// --------------------------------------------------------

void cmd_read_flash_id()
{
	uint8_t buffer[17] = { 0x9F, /* zeros */ };
	flashio(buffer, 17, 0);

	for (int i = 1; i <= 16; i++) {
	  printf(" %x", buffer[i]);
	}
	putchar('\n');
}

// --------------------------------------------------------

#ifdef HX8KDEMO
uint8_t cmd_read_flash_regs_print(uint32_t addr, const char *name)
{
	set_flash_latency(8);

	uint8_t buffer[6] = {0x65,
			     (uint8_t)(addr >> 16), (uint8_t)(addr >> 8),
			     (uint8_t)(addr), 0, 0};
	flashio(buffer, 6, 0);

	printf("0x%x &s 0x%x\n", addr, name, buffer[5]);

	return buffer[5];
}

void cmd_read_flash_regs()
{
	print("\n");
	uint8_t sr1v = cmd_read_flash_regs_print(0x800000, "SR1V");
	uint8_t sr2v = cmd_read_flash_regs_print(0x800001, "SR2V");
	uint8_t cr1v = cmd_read_flash_regs_print(0x800002, "CR1V");
	uint8_t cr2v = cmd_read_flash_regs_print(0x800003, "CR2V");
	uint8_t cr3v = cmd_read_flash_regs_print(0x800004, "CR3V");
	uint8_t vdlp = cmd_read_flash_regs_print(0x800005, "VDLP");
}
#endif

#if defined(ICEBREAKER) || (ULX3S)
uint8_t cmd_read_flash_reg(uint8_t cmd)
{
	uint8_t buffer[2] = {cmd, 0};
	flashio(buffer, 2, 0);
	return buffer[1];
}

void print_reg_bit(int val, const char *name)
{
	for (int i = 0; i < 12; i++) {
		if (*name == 0)
			putchar(' ');
		else
			putchar(*(name++));
	}

	putchar(val ? '1' : '0');
	putchar('\n');
}

void cmd_read_flash_regs()
{
	putchar('\n');

	uint8_t sr1 = cmd_read_flash_reg(0x05);
	uint8_t sr2 = cmd_read_flash_reg(0x35);
	uint8_t sr3 = cmd_read_flash_reg(0x15);

	print_reg_bit(sr1 & 0x01, "S0  (BUSY)");
	print_reg_bit(sr1 & 0x02, "S1  (WEL)");
	print_reg_bit(sr1 & 0x04, "S2  (BP0)");
	print_reg_bit(sr1 & 0x08, "S3  (BP1)");
	print_reg_bit(sr1 & 0x10, "S4  (BP2)");
	print_reg_bit(sr1 & 0x20, "S5  (TB)");
	print_reg_bit(sr1 & 0x40, "S6  (SEC)");
	print_reg_bit(sr1 & 0x80, "S7  (SRP)");
	putchar('\n');

	print_reg_bit(sr2 & 0x01, "S8  (SRL)");
	print_reg_bit(sr2 & 0x02, "S9  (QE)");
	print_reg_bit(sr2 & 0x04, "S10 ----");
	print_reg_bit(sr2 & 0x08, "S11 (LB1)");
	print_reg_bit(sr2 & 0x10, "S12 (LB2)");
	print_reg_bit(sr2 & 0x20, "S13 (LB3)");
	print_reg_bit(sr2 & 0x40, "S14 (CMP)");
	print_reg_bit(sr2 & 0x80, "S15 (SUS)");
	putchar('\n');

	print_reg_bit(sr3 & 0x01, "S16 ----");
	print_reg_bit(sr3 & 0x02, "S17 ----");
	print_reg_bit(sr3 & 0x04, "S18 (WPS)");
	print_reg_bit(sr3 & 0x08, "S19 ----");
	print_reg_bit(sr3 & 0x10, "S20 ----");
	print_reg_bit(sr3 & 0x20, "S21 (DRV0)");
	print_reg_bit(sr3 & 0x40, "S22 (DRV1)");
	print_reg_bit(sr3 & 0x80, "S23 (HOLD)");
	putchar('\n');
}
#endif

// --------------------------------------------------------

uint32_t cmd_benchmark(bool verbose, uint32_t *instns_p)
{
	uint8_t data[256];
	uint32_t *words = (uint32_t*)data;

	uint32_t x32 = 314159265;

	uint32_t cycles_begin, cycles_end;
	uint32_t instns_begin, instns_end;
	RDCYCLE(cycles_begin);
	RDINSTR(instns_begin);

	for (int i = 0; i < 20; i++)
	{
		for (int k = 0; k < 256; k++)
		{
			x32 ^= x32 << 13;
			x32 ^= x32 >> 17;
			x32 ^= x32 << 5;
			data[k] = x32;
		}

		for (int k = 0, p = 0; k < 256; k++)
		{
			if (data[k])
				data[p++] = k;
		}

		for (int k = 0, p = 0; k < 64; k++)
		{
			x32 = x32 ^ words[k];
		}
	}

	RDCYCLE(cycles_end);
	RDINSTR(instns_end);


	if (verbose)
	{
	        printf("Cycles: 0x%x\n", cycles_end - cycles_begin);
		printf("Instns: 0x%x\n", instns_end - instns_begin);
		printf("Chksum: 0x%x\n", x32);
	}

	if (instns_p)
		*instns_p = instns_end - instns_begin;

	return cycles_end - cycles_begin;
}

// --------------------------------------------------------

#ifdef HX8KDEMO
void cmd_benchmark_all()
{
	uint32_t instns = 0;

	print("default        ");
	reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00000000;
	printf(": %x\n", cmd_benchmark(false, &instns));

	for (int i = 8; i > 0; i--)
	{
	        printf("dspi-%d         ", i);
	        set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00400000;

		printf(": %x\n", cmd_benchmark(false, &instns));
	}

	for (int i = 8; i > 0; i--)
	{
	        printf("dspi-crm-%d     ", i);;

		set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00500000;

		printf(": %x\n", cmd_benchmark(false, &instns));
	}

	for (int i = 8; i > 0; i--)
	{
	        printf("qspi-%d         ", i);

		set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00200000;

		printf(": %x\n", cmd_benchmark(false, &instns));
	}

	for (int i = 8; i > 0; i--)
	{
	        printf("qspi-crm-%d     ", i);

		set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00300000;

		printf(": %x\n", cmd_benchmark(false, &instns));
	}

	for (int i = 8; i > 0; i--)
	{
	        printf("qspi-ddr-%d     ", i);
		
		set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00600000;

		printf(": %x\n", cmd_benchmark(false, &instns));
	}

	for (int i = 8; i > 0; i--)
	{
	        printf("qspi-ddr-crm-%d ", i);
		
		set_flash_latency(i);
		reg_spictrl = (reg_spictrl & ~0x00700000) | 0x00700000;

		printf(": %x\n",  cmd_benchmark(false, &instns));
	}

	print("instns         ");
	printf(": %x\n", instns);
}
#endif

#if defined(ICEBREAKER) || defined (ULX3S)
void cmd_benchmark_all()
{
	uint32_t instns = 0;

	print("default   ");
	set_flash_mode_spi();
	printf("%x\n", cmd_benchmark(false, &instns));

	print("dual      ");
	set_flash_mode_dual();
	printf("%x\n", cmd_benchmark(false, &instns));
	
	// print("dual-crm  ");
	// enable_flash_crm();
	// printf("%x\n", cmd_benchmark(false, &instns));

	print("quad      ");
	set_flash_mode_quad();
	printf("%x\n", cmd_benchmark(false, &instns));
	
	print("quad-crm  ");
	enable_flash_crm();
	printf("%x\n", cmd_benchmark(false, &instns));
	
	print("qddr      ");
	set_flash_mode_qddr();
	printf("%x\n", cmd_benchmark(false, &instns));
	
	print("qddr-crm  ");
	enable_flash_crm();
	printf("%x\n", cmd_benchmark(false, &instns));
}
#endif

void cmd_echo()
{
	print("Return to menu by sending '!'\n\n");
	char c;
	while ((c = getchar()) != '!')
		putchar(c);
}


const uint8_t vgmfile[] = {
};

uint8_t getbyte() {
  int32_t c = -1;
  while (c == -1) {
    c = reg_uart_data;
  }
  return c;
}

const uint16_t sn_vol_tab[16] = {
  32767, 26028, 20675, 16422, 13045, 10362,  8231,  6568,
  5193,  4125,  3277,  2603,  2067,  1642,  1304,     0
};

int highmemsize = 0;

void cmd_dac(uint8_t alt)
{
  print("DAC & FM/VGM testing\n");
  
  const uint32_t* audiofile = (const uint32_t*)sdram;
  //{};

  uint8_t* highmem = (uint8_t*)audiofile;

  //for (int i = 0; i < 16; ++i)  sdram[i] = i;
  for (int i = 0; i < 16; ++i)
    printf("%02x ", highmem[i]);
  for (int i = 0; i < 16; ++i)
    printf("%c", highmem[i]);

/*
  uint16_t c;
#if 1
  while (c = getbyte())
    reg_dac = (c << 8); // scale to 16-bit, ...
#else
  for (uint16_tc = 0; 1; ++c)
    reg_dac = c;
#endif
*/

  // volume | freq / half period length
  //reg_fm[0] = (0x1000 << 16) | (100/2); // fm0
  //reg_fm[1] = (0x800 << 16) | (100/2); // fm1 440
  //reg_fm[2] = (0x7000 << 16) | (882/2); // fm2
  //reg_fm[3] = (0x8000 << 16) | (alt ? 0x8000 | 16*80: 16*d120); // fm3 noise
  
  uint32_t audiosize = highmemsize / sizeof(*audiofile); // sizeof(audiofile)
  for (int i = 0; i < audiosize; ++i) {
    uint32_t sample = audiofile[i];
    reg_dac[0] = /*bswap_16*/(sample & 0xffff); // left
    reg_dac[1] = /*bswap_16*/(sample >> 16); // right
  }
  //for (int i = 0; 1; ++i) reg_dac = i & 1 ? 0xffff : 0;
  reg_dac[0] = 0; // silence DAC
  
  uint32_t offset = *(uint32_t*)(vgmfile + 0x34);
  printf("%.4s %d %x %d\n", vgmfile, offset,
	 *(uint32_t*)(vgmfile + 0x08),
	 3579545);
  //*(uint32_t*)(vgmfile + 0x0c));
  
  // samples @ 44.1kHz, yay!
  uint8_t reg = 0;
  uint16_t regs[8] = {}; // tone, volume, ...
  
  const uint8_t verbose = 0;
  
  uint32_t cycles_begin;
  
  offset = offset ? offset : 0xc;
  for (int i = 0x34 + offset; i < sizeof(vgmfile); ++i) {
    int wait = 0;
    uint8_t c = vgmfile[i];
    
    if (c == 0x50) { // SN76489/96 register write
      c = vgmfile[++i];
      if (verbose)
	printf("SN: %x\n", c);
      
      // SN76489 has 8 "registers":
      // 4 x 4 bit volume registers plus:
      // 3 x 10 bit tone registers & 1 x 3 bit noise register
      
      if (c & 0x80) { // LATCH?
	reg = (c >> 4) & 0b111;
	regs[reg] = (regs[reg] & 0xff0) | (c & 0xf);
      } else {
	if (reg & 1) { // 4-bit volume
	  regs[reg] = c & 0xf; // a bit superflous?
	} else { // 10-bit tone register
	  if (reg == 0b110) // noise?
	    regs[reg] = c & 0x3f; // always writes the LSB
	  else
	    regs[reg] = (regs[reg] & 0x00f) | ((c & 0x3f) << 4);
	}
      }
      
      if (verbose)
	printf("REG[%d, %s]: %d %dHz %d/db/\n",
	       (reg >> 1), (reg & 1 ? "VOL" : "FRQ"),
	       regs[reg], 3579545 / 16 / MAX(regs[reg & 0xe], 1),
	       -regs[reg | 1]);
      
      // translate to our mmio FM registers
      const uint8_t ch = (reg >> 1);
      if (reg & 1) { // volume
	uint16_t vol = sn_vol_tab[regs[reg]];
	reg_fm16[ch * 2 + 1] = vol;
      } else {
	uint16_t frq = regs[reg];
	
	if (ch == 3) {
	  if (verbose) printf("noise: %x", frq);
	  uint8_t white = (frq >> 3) & 1; // periodic, or white noise?
	  switch (frq & 0x3) {
	  case 0: frq = 0x10; break;
	  case 1: frq = 0x20; break;
	  case 2: frq = 0x40; break;
	  case 4:
	    printf("NYI ch2 sync\n");
	    frq = 0x30; break;
	  };
	  frq = 3579545 / 16 / 2 / frq; // SN to Hz
	  frq = 16 * 44100 / 2 / frq * 2; // Hz to our samples
	  
	  if (verbose) printf(" > %x, %x, %d\n", frq, white);
	  if (!white) frq |= 0x8000; // periodic noise bit
	} else {
	  if (frq) frq = 3579545 / 16 / 2 / frq;
	  if (verbose) printf("ch: %d f: %d\n", ch, frq);
	  frq = 16 * 44100 / 2 / frq;
	}
	//if (ch == 3)
	reg_fm16[ch * 2] = frq;
      }
    } else if (c == 0x61) {
      wait = (uint8_t)vgmfile[++i] + vgmfile[++i] * 256;
    } else if (c == 0x62) {
      wait = 735; // wait 735 samples
    } else if (c == 0x63) {
      wait = 882; // wait 882 samples
    } else if (c >= 0x70 && c <= 0x7f) {
      wait = c - 0x70 + 1; // wait n+1 samples
    } else if (c == 0x66) {
      printf("EOF\n");
      break;
    } else {
      printf("NYI: %x\n", c);
    }
    
    if (wait) {
      //printf("wait: %d\n", wait);
      wait = SYSCLK / 44100 * wait;
      uint32_t cycles_now;
      do {
	RDCYCLE(cycles_now);
      } while (cycles_now - cycles_begin < wait);
    }
    // update after reach command to help compensate expensive div delay?
    RDCYCLE(cycles_begin);
  }
  
 return_silence:
  reg_dac[0] = 0; // silence DAC

  // disable all FM ops that might be left running:
  for (uint8_t i = 0; i < 4; ++i)
    reg_fm[i] = 0;
}

uint8_t* midifile;
int midifilesize = 0;

uint32_t midi_readVar(int& offset)
{
  uint32_t ret = 0;
  int v;
  do {
    v = midifile[offset++];
    if (v == -1) break;

    ret = (ret << 7) | (v & 0x7f);
  } while (v & 0x80);
  
  return ret;
}

void midi_sel_instr(uint8_t ch, uint8_t instr)
{
  reg_uart_midi = 0xc0 | ch;
  reg_uart_midi = instr;
}

void midi_note(uint8_t ch, uint8_t note, uint8_t vol = 0)
{
  reg_uart_midi = 0x90 | ch;
  reg_uart_midi = note;
  reg_uart_midi = vol;
}

struct MidiHeader {
  char typ[4];
  EndianessConverter<uint32_t, BigEndianTraits> length;
};

struct MidiHeaderChunk {
  EndianessConverter<uint16_t, BigEndianTraits> format;
  EndianessConverter<uint16_t, BigEndianTraits> tracks;
  EndianessConverter<uint16_t, BigEndianTraits> division;
};

void cmd_midi()
{
  print("midi file to load> ");
  
  uint8_t midi_ch = 1;
  uint8_t midi_vol = 0x70;
  
  // select instrument
  midi_sel_instr(midi_ch, 0x41);
  
  // test notes on channel 1
  midi_note(midi_ch, 0x40, midi_vol);
  midi_note(midi_ch, 0x43, midi_vol);
  
  // drum note on channel 10 (9)
  midi_note(9, 0x23, 0x70); // accustic bass
  midi_note(9, 0x23);
  
  char filename[18];
  getline(filename, sizeof(filename));
  
  // note off
  midi_note(midi_ch, 0x40);
  midi_note(midi_ch, 0x43);
  
  
  int file = open(filename);
  if (!file) {
    printf("could not open\n");
    return;
  }
  
  printf("MIDI file: %d\n", file);
  midifile = (uint8_t*)sdram;
  midifilesize = read(file, midifile, 1024*1024);
  close(file); file = 0;
    
  // MIDI file
  int offset = 0;
  int division = 480; // ticks per quater
  int tempo = 120;
  
  int _tracks;
  struct track {
    int offset;
    int tdelta;
  };
  track tracks[32];
  uint8_t runcmds[sizeof(tracks) / sizeof(*tracks)] = {};
  uint8_t runtrack = 0xff;

  // parse main chunk
  MidiHeader* header = (MidiHeader*)(&midifile[0] + offset);
  offset += sizeof(MidiHeader);
  printf("> %.4s %d\n", header->typ, *header->length);

  if (strncmp(header->typ, "MThd", 4) != 0) {
    printf("not a midi file\n");
    return;
  }
  
  // parse and setup all tracks
  {
    MidiHeaderChunk* headerchunk = (MidiHeaderChunk*)(&midifile[0] + offset);
    offset += sizeof(MidiHeaderChunk);
    printf("MThd> %d %d %d\n",
	   *headerchunk->format, *headerchunk->tracks, *headerchunk->division);
    
    header = (MidiHeader*)(&midifile[0] + offset);
    division = *headerchunk->division;
    _tracks = *headerchunk->tracks;
    if (_tracks > sizeof(tracks) / sizeof(*tracks)) {
      printf("Too many tracks!\n");
      _tracks = sizeof(tracks) / sizeof(*tracks);
    }
    
    for (int i = 0; i < _tracks; ++i) {
      if (strncmp(header->typ, "MTrk", 4) != 0) {
	printf("expected a track! got: %.4s\n", header->typ);
	break;
      }
      printf("MTrk %x %x\n", offset, *header->length);
      offset += sizeof(MidiHeader);
      tracks[i].offset = offset;
      tracks[i].tdelta = -1;
      offset += header->length;
      header = (MidiHeader*)(&midifile[0] + offset);
    }
  }
  
  uint32_t cycles_begin;
  RDCYCLE(cycles_begin);
  
  const bool verbose = false;
  int len = 1; // determine len from event
  uint8_t mintrack = 0, maxtrack = _tracks - 1;
  bool allend = false;
  while (!allend) {
    int16_t e; // event, fwd.
    allend = true;
    
    // find next active channel event
    int track, tdelta = 0x7fffffff;
    if (verbose) printf("\nNext tdeltas:");
    for (int i = mintrack; i <= maxtrack; ++i) {
      // valid track from file?
      if (!tracks[i].offset) continue;
      
      if (verbose) printf(" %d", tracks[i].tdelta);
      
      allend = false;
      // find next event
      if (tracks[i].tdelta < tdelta) {
	track = i; // track and file offset to use
	tdelta = tracks[i].tdelta;
	offset = tracks[i].offset;
      }
    }
    if (verbose) printf("\n");
    
    if (allend) break;
    
  restart:
    
    printf("\ttr%X @%x t%x>", track, offset, tdelta);
    
    // 1st tdelta not initialized?
    if (tdelta == -1) {
      goto next;
    } else if (tdelta > 0) {
      uint32_t wait = 60 * SYSCLK / tempo / division * tdelta;
      
      // update all track next timing deltas
      for (int i = mintrack; i <= maxtrack; ++i) {
	// valid track from file?
	if (tracks[i].offset)
	  tracks[i].tdelta -= tdelta;
      }

      // key to exit
      if (reg_uart_data != -1)
	break;

      uint32_t cycles_now;
      do {
	RDCYCLE(cycles_now);
      } while (cycles_now - cycles_begin < wait);
      cycles_begin = cycles_now; // next reference
    } else if (tdelta < -1) {
      printf("Bug: in timing deltas!\n");
      return;
    }
    
    e = midifile[offset++];
    if (e < 0x80) {
      --offset; // re-read below
      
      // if not the same track, re-issue command!
      if (track != runtrack) {
	reg_uart_midi = runcmds[track];
      }
      goto running_status;
    }
    
    len = 1;
    switch (e & 0xf0) {
    case 0x80: // note off
    case 0x90: // note on
    case 0xa0: // polyphonic key presssure / Aftertouch
    case 0xb0: // control change
    case 0xe0: // pitch bend
      len = 2; break;
      
    case 0xc0: // program change, instrument / sound
    case 0xd0: // channel key pressure
      len = 1; break;

    case 0xf0:
      switch(e) {
      case 0xf0: // sysex
	len = midi_readVar(offset);
	printf(" sysex: %d\n", len);
#if 1
	// run it, what could possibly go wrong!
#else
	for (int i = 0; i < len; ++i)
	  printf(" %02x", midifile[offset + i]);
	printf("\n");
	offset += len;
	goto next;
#endif
	break;
      case 0xf7: // sysex
	printf(" sysex: END? (should be in SYSEX len!)\n");
	goto next;
	break;

      case 0xff: // meta events
	{
	  uint8_t typ = midifile[offset++];
	  len = midi_readVar(offset);
	  const char* str = "UNKN";
	  bool _str = 1;
	  switch (typ) {
	  case 0: str = "SEQ#"; break;
	  case 1: str = "TEXT"; break;
	  case 2: str = "COPYR"; break;
	  case 3: str = "NAME"; break;
	  case 4: str = "INSTR"; break;
	  case 5: str = "LYRIC"; break;
	  case 6: str = "MARKER"; break;
	  case 7: str = "CUE"; break;
	  case 8: str = "PROGNAME"; break;
	  case 9: str = "DEVNAME"; break;
	  case 0x20: str = "MIDIPREFIX"; break;
	  case 0x21: str = "MIDIPORT"; break;
	  case 0x2f:
	    str = "EOT";
	    offset = 0;
	    break;
	  case 0x51: str = "TEMPO";
	     // µs per quarter note
	    _str = 0;
	    tempo = 60 * 1000000 /
	      ((midifile[offset + 0] << 16) |
	       (midifile[offset + 1] << 8) |
	       midifile[offset + 2]);
	    printf("New tempo: %d\n", tempo);
	    break;
	  case 0x54: str = "SMTPE"; break;
	  case 0x58: str = "TIME"; _str = 0; break;
	  case 0x59: str = "KEYSIG"; _str = 0; break;
	  case 0x7f: str = "SEQSPEC"; _str = 0; break;
	  }
	  
	  printf(" meta: %02x %s, #%d", typ, str, len);
	  if (_str)
	    printf(": %.*s\n", len, &midifile[0] + offset);
	  else {
	    for (int i = 0; i < len; ++i)
	      printf(" %02x", midifile[offset + i]);
	    printf("\n");
	  }
	  
	  offset += len;
	  goto next;
	}
      default:
	printf(" NYI: EXT: %02x!\n", e);
      }
      break;
      
    default:
      printf(" NYI: MIDI: %02x!\n", e);
      break;
    };
    
    printf("\t%02x", e);
    // shift channels, for mt32 2-8+10 mapping :-/
    //if ((e < 0xf0) && (e & 0xf) != 9) e += 1;
    
    // cache for running status commands
    reg_uart_midi = e;
    runcmds[track] = e;
  
  running_status:
    runtrack = track;
    
    for (int i = 0; i < len; ++i, ++offset) {
      // TODO: sanity check MSB not set!
      //printf(" %02x", midifile[offset]);
      reg_uart_midi = midifile[offset];
    }
    
    //printf("."); // was "\n"
    
  next:

    // read next time delta
    if (offset) { // not EOT
      tdelta = midi_readVar(offset);
      
      // optimize, if next tdelta == 0 directly restart
      if (tdelta == 0)
	goto restart;
    }
    
    // update track tdelta & offset
    tracks[track].tdelta = tdelta;
    tracks[track].offset = offset;
  }
  
  return;
}

void cmd_read_ram()
{
  printf("0> %x %x %x %x %x\n", sdram[0], sdram[1], sdram[2], sdram[3], sdram[4]);

  sdram[0] = 0x00112233; sdram[1] = 0x44556677; sdram[2] = 0x8899aabb; sdram[3] = 0xccddeeff;
  //sdram[0] = 0x0011; sdram[1] = 0x2233; sdram[2] = 0x4455; sdram[3] = 0x6677;
  printf("1> %x %x %x %x\n", sdram[0], sdram[1], sdram[2], sdram[3]);
  printf("2> %x %x %x %x\n", sdram[0], sdram[1], sdram[2], sdram[3]);

  sdram[0] = 0xffeeddcc; sdram[1] = 0xbbaa9988; sdram[2] = 0x77665544; sdram[3] = 0x33221100;
  //sdram[0] = 0xffee; sdram[1] = 0xddcc; sdram[2] = 0xbbaa; sdram[3] = 0x9988;
  printf("3> %x %x %x %x\n", sdram[0], sdram[1], sdram[2], sdram[3]);
  printf("4> %x %x %x %x\n", sdram[0], sdram[1], sdram[2], sdram[3]);

  printf("++16> %x %x %x %x\n", ++sdram[0], ++sdram[1], ++sdram[2], ++sdram[3]);
  printf("++16> %x %x %x %x\n", ++sdram[0], ++sdram[1], ++sdram[2], ++sdram[3]);

  //printf(" 8> %x %x %x %x", vga_vram8[0], vga_vram8[1], vga_vram8[4+0], vga_vram[4+1]);
  //printf("\n32> %x %x %x %x", vga_vram32[0], vga_vram32[1], vga_vram32[2], vga_vram32[3]);
  //printf(" 32++> %x %x %x %x", ++vga_vram32[0], ++vga_vram32[1], ++vga_vram32[2], ++vga_vram[3]);

  printf("\n");
}


void cmd_analyze()
{
  volatile uint32_t* log_buffer = (volatile uint32_t*)0x10000000;
  int pos = 0;
  while (pos >= 0) {
    uint32_t samples = log_buffer[2048];
    printf("@ %d/%d samples\n", pos, samples);
    for (int ch = 0; ch < 4; ++ch) {
      switch (ch) {
      case 0: print("C:"); break;
      case 1: print("S:"); break;
      case 2: print("O:"); break;
      case 3: print("I:"); break;
      default: print("X:"); break;
     }
      for (int i = 0; i < 80 && pos + i < samples; ++i) {
	// this is single cycle SRAM, so no point in caching, ...
	uint32_t v = log_buffer[pos + i];
	print(v & (1 << ch) ? "^" : "_");
      }
      print("\n");
    }
    
    char c = getchar();
    switch (c) {
    case '1':
      pos = 0; break;
    case ' ':
    case '\r':
    case '\n':
      pos += 40; break;
    case 'b':
    case '\b':
    case '\x7f':
      pos -= 40; if (pos < 0) pos = 0; break;
    default:
      printf("Quit w/ %d\n", c);
    case 'q':
      pos = -1;
    }
  }
}

const unsigned char crc7poly = 0b10001001;
uint8_t crc7(const uint8_t* msg, uint16_t n) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < n; ++i) {
     crc ^= msg[i];
     for (uint8_t j = 0; j < 8; ++j) {
      // crc = crc & 0x1 ? (crc >> 1) ^ crc7poly : crc >> 1;
      crc = (crc & 0x80u) ? ((crc << 1) ^ (crc7poly << 1)) : (crc << 1);
    }
  }
  return crc;
}

volatile uint32_t* spimmc = (volatile uint32_t*)0x02000010; // full 32 bits
volatile uint16_t* spimmc16 = (volatile uint16_t*)0x02000012; // first 16 MSBs
volatile uint8_t* spimmc8 = (volatile uint8_t*)0x02000013; // first 8 MSBs
volatile uint8_t* spimmcOOB = (volatile uint8_t*)0x02000011; // invalid trunctated write

// TODO: better SD status / error handling

#define BIT(x) (1 << x)

const bool sd_log = false;

uint8_t sd_status() {
  uint8_t status;
  // wait for status
  if (sd_log) printf("status: ");
  for (int i = 0; i < 16; ++i) {
    status = *spimmc;
    if (sd_log) printf(" %x", status);
    // wait for stop bit to clear
    if ((status & 0x80) == 0)
      break;
  }
  if (status && status != 0xff) {
    printf(":", status);
    printf("%s%s%s%s%s%s%s\n",
	   status & BIT(6) ? " ParameterError" : "",
	   status & BIT(5) ? " AddressError" : "",
	   status & BIT(4) ? " EraseSeqError" : "",
	   status & BIT(3) ? " CommandCRCReror" : "",
	   status & BIT(2) ? " IllegalCommand" : "",
	   status & BIT(1) ? " EraseReset" : "",
	   status & BIT(0) ? " Idle" : "");
    if (!sd_log) printf("\n");
  } else {
    if (sd_log) printf("\n");
  }
  
  return status;
}

uint8_t sd_cmd(uint8_t* cmd, int size, uint8_t* data = 0, int dsize = 0,
	       bool nodeassert = false) {
  cmd[0] |= 0x40;
  cmd[size - 1] = crc7(cmd, size - 1) | 1; // stop bit
  for (int i = 0; i < size; ++i) {
    *spimmc8 = cmd[i];
    if (sd_log) printf("%02x", cmd[i]);
  }
  if (sd_log) printf(" - ");
  uint8_t status = sd_status();
  // response payload
  if (dsize > 0 && (status == 0 || status == 1)) {
    for (int i = 0; i < dsize; ++i) {
      data[i] = *spimmc;
      if (sd_log) printf("%02x", data[i]);
    }
    if (sd_log) printf("\n");
  }
  
  if (!nodeassert)
    *spimmcOOB = 0xff; // OOB de-assert CS, start new command

  return status;
}

uint8_t sd_acmd(uint8_t* acmd, int size, uint8_t* data, int dsize) {
  // 1st send CMD55 ACMD prefix command
  uint8_t cmd55[6] = {55};
  uint8_t status = sd_cmd(cmd55, sizeof(cmd55), 0, 0, true);
  if (status != 0x01 && status != 0x05) {
    printf("CMD55 error\n");
    return status;
  }

  status = sd_cmd(acmd, size, data, dsize);
  if (status != 0x01 && status != 0x00) {
    printf("ACMD error\n");
    return status;
  }
  
  return status;
}

uint8_t sd_read(uint32_t addr, uint8_t* data, int dsize) {
  static const int blocksize = 512;
  uint8_t status = 0;
  
  for (; dsize > 0; dsize -= blocksize, ++addr) {
    uint8_t cmd17[6] = {17,
			(uint8_t)(addr >> 24),(uint8_t)(addr >> 16),
			(uint8_t)(addr >> 8), (uint8_t)(addr)};
    status = sd_cmd(cmd17, sizeof(cmd17), 0, 0, true);
    
    // wait for data token
    uint8_t token;
    for (int i = 0; i < 128; ++i) {
      token = *spimmc;
      if (token == 0b11111110)
	break;
      else if (token != 0xff)
	printf("Unknown token: %02x\n", token);
      token = 0;
    }
    if (token) {
      for (int i = 0; i < blocksize; ++i, ++data) {
	*data = *spimmc;
	if (sd_log) printf("%02x", *data);
      }
      if (sd_log) printf("\n");
    }
    
    uint16_t crc = (*spimmc) << 16 | *spimmc;
    // TODO: check crc
    if (sd_log) printf("crc: %04x\n", crc);
    
    *spimmcOOB = 0xff; // OOB de-assert CS, start new command
  }

  return status;
}


// +218 0x0000
// +220 0x00		// orig. drive
// +221 0x00		// sec
// +221 0x00		// min
// +222 0x00		// hour
// +440 0x00000000	// disk id
// +446 // part 1
// +462 // part 2
// +478 // part 3
// +494 // part 4
// +510 // signature 0x55aa

struct MbrPart {
  uint8_t status;
  uint8_t first_chs[3];
  uint8_t type;
  uint8_t last_chs[3];
  EndianessConverter<uint32_t, LittleEndianTraits> first_lba;
  EndianessConverter<uint32_t, LittleEndianTraits> sectors;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;

struct BiosParameterBlock {
  // DOS 2.0
  EndianessConverter<uint16_t, LittleEndianTraits> bytesPerSector;
  uint8_t sectorsPerCluster;
  EndianessConverter<uint16_t, LittleEndianTraits> reservedSectors;
  uint8_t fats;
  EndianessConverter<uint16_t, LittleEndianTraits> maxRootEntries;
  EndianessConverter<uint16_t, LittleEndianTraits> totalSectors;
  uint8_t media_desc;
  EndianessConverter<uint16_t, LittleEndianTraits> sectorsPerFAT;
  
  // DOS 3.31
  EndianessConverter<uint16_t, LittleEndianTraits> sectorsPerTrack;
  EndianessConverter<uint16_t, LittleEndianTraits> heads;
  EndianessConverter<uint32_t, LittleEndianTraits> hiddenSectors;
  EndianessConverter<uint32_t, LittleEndianTraits> totalSectorsAndHidden;
  
  // FAT32 Extended
  EndianessConverter<uint32_t, LittleEndianTraits> sectorsPerFAT2;
  EndianessConverter<uint16_t, LittleEndianTraits> driveDesc;
  EndianessConverter<uint16_t, LittleEndianTraits> version;

  EndianessConverter<uint32_t, LittleEndianTraits> rootCluster;
  EndianessConverter<uint16_t, LittleEndianTraits> infoSector;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;


struct FatBootSector {
  uint8_t jump[3];
  char oemname[8];
  BiosParameterBlock bpb;
  uint8_t pad[499 - sizeof(BiosParameterBlock)];
  EndianessConverter<uint16_t, LittleEndianTraits> signature;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;

struct FatFsInfo {
  uint8_t pad[512];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;


struct FatDirEntry {
  char filename[8];
  char ext[3];

  uint8_t attributes;
  uint8_t pad[8];
  
  EndianessConverter<uint16_t, LittleEndianTraits> highCluster;
  EndianessConverter<uint16_t, LittleEndianTraits> modTime;
  EndianessConverter<uint16_t, LittleEndianTraits> modDate;
  EndianessConverter<uint16_t, LittleEndianTraits> startCluster;
  EndianessConverter<uint32_t, LittleEndianTraits> fileSize;
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;

struct VFatDirEntry {
  uint8_t sequence;
  char /*EndianessConverter<uint16_t, LittleEndianTraits>*/ name1[5*2];
  uint8_t attributes;
  uint8_t type;
  uint8_t dosNameChksum;
  char /*EndianessConverter<uint16_t, LittleEndianTraits>*/ name2[6*2];
  EndianessConverter<uint16_t, LittleEndianTraits> startCluster;
  char /*EndianessConverter<uint16_t, LittleEndianTraits>*/ name3[2*2];
}
#ifdef __GNUC__
__attribute__((packed))
#endif
;

// internal housekeeping
struct FAT {
  uint32_t partlba;
  uint32_t fat;
  uint32_t rootCluster;
  uint32_t clusterSize;
  uint32_t dataStart;
};

// TODO: ClusterMap bits, of 12, 16, 32 ... little endian

uint32_t cached = 0;
uint8_t block[512/**2*/]; // TODO: one temp. test "cluster" and dynamic cluster size!
uint8_t block2[512/**2*/]; // TODO: one temp. test "cluster" and dynamic cluster size!

static const bool fat_log = false;

uint8_t fat_read_cluster(FAT* fat, uint32_t cluster, uint8_t* data, int dlen) {
  // TODO: check cluster >= 2! // 0 and 1 reserverd, starts at 2!
  uint32_t addr = fat->fat + fat->dataStart + (cluster - 2) * fat->clusterSize;
  if (fat_log)
    printf("read cluster: %x, start: %d @ %d\n", cluster, fat->fat + fat->dataStart, addr);
  uint8_t status = sd_read(fat->partlba + addr, data, dlen);
  if (status) {
    printf("fat_read_cluster: error: %x\n", status);
  }
  return status;
}

uint32_t fat_next_cluster(FAT* fat, uint32_t cluster) {
  // decode FAT allocation
  uint32_t addr = (cluster * 4) / 512; // TODO: optimized & readable div & rem
  uint32_t off = cluster * 4 - addr * 512;
  if (fat_log) printf("FAT@: %d %d, for cluster: %d\n", addr, off, cluster);
  addr += fat->fat;
  if (addr == cached) {
    if (fat_log) printf("cached: %x\n", cached);
  } else {
    uint8_t status = sd_read(fat->partlba + addr, block2, sizeof(block2));
    if (status) {
      printf("fat_next_cluster: error: %x\n", status);
      return 0;
    };
    cached = addr;
  }

  // test print decode first cluster allocation
  uint32_t* le32 = (uint32_t*)block2;
  if (fat_log) {
    printf("clusters:");
    for (int i = 0; i < 512 / 4; ++i) {
      printf(" %x", le32[i]);
      if (i % 16 == 15) printf("\n");
    }
  }
  
  uint32_t ret = le32[off / 4] & 0xfffffff; // only 28 bit valid
  if (fat_log) printf("should ret: %x\n", ret);
  
  // defect / free / end of list?
  return (ret > 0xffffff8) ? 0 : ret;
}

FAT fat;
uint32_t fddb; // File Descriptor DB

char* rmstrtrail(char* str, char ch)
{
  int i;
  for (i = strlen(str); i > 0;) {
    if (str[i-1] != ch) break;
    --i;
    str[i] = 0;
  }
  return str + i;
}

int open(const char* pathname, int flags)
{
  char filename[8+4+1];
  printf("open\n");
  
  // decode root directory, TODO: descent into chils directories
  for (uint32_t cluster = fat.rootCluster; cluster;) {
    int status = fat_read_cluster(&fat, cluster, block, sizeof(block));
    if (status) return -1; // TODO: set errno
    
    FatDirEntry* dentry = (FatDirEntry*)block;
    VFatDirEntry* ventry = (VFatDirEntry*)block;
    for (int i = 0; i < sizeof(block) / sizeof(FatDirEntry); ++i) {
      if (dentry[i].attributes == 0x0f)
	printf("%d %d %.9s%.9s%.4s\n", i, ventry[i].sequence,
	       ventry[i].name1, ventry[i].name2, ventry[i].name3);
      else {
	printf("%d ", i);
	
	switch (dentry[i].filename[0]) {
	case 0: printf("EOF NULL\n"); i = sizeof(block) / sizeof(FatDirEntry); break;
	case 0xe5: printf("DELETED ");
	case 0x2e: printf("DOT Entry ");
	default:
	  //printf("%.8s.%.3s",  dentry[i].filename, dentry[i].ext);
	  sprintf(filename, "%.8s", dentry[i].filename);
	  sprintf(rmstrtrail(filename, ' '), ".%.3s", dentry[i].ext);
	  rmstrtrail(filename, ' ');
	  printf("'%s'", filename);
	  break;
	}
	
	if (dentry[i].attributes & 0x10)
	  printf(" <DIR>");
	
	if (strncasecmp(filename, pathname, 8+4) == 0) {
	  printf(" !! MATCH !!\n");
	  // TODO: register and return fd dictionary
	  fddb = (dentry[i].highCluster << 16) | dentry[i].startCluster;
	  return fddb;
	}
	printf("\n");
      }
    }
    
    cluster = fat_next_cluster(&fat, cluster);
  }
  
  return 0;
}

size_t _read(int fd, void* buf, size_t count)
{
  //printf("read\n");

  // eof?
  if (!fddb)
    return -1;
  
  // TODO: support abritrary unaligned sizes
  int status = fat_read_cluster(&fat, fddb, (uint8_t*)buf, count);
  if (status)
    return -1;
  
  fddb = fat_next_cluster(&fat, fddb);
  
  return count;
}

size_t read(int fd, void* _buf, size_t count)
{
  char* buf = (char*)_buf;
  size_t ret = 0, total = 0;
  while (ret >= 0 && total < count) {
    printf(".");
    ret = _read(fd, buf + total, sizeof(block));
    if (ret >= 0) {
      total += sizeof(block);
    }
  }
  printf("EOF: %s\n", total);
  return total;
}

int close(int fd)
{
  //printf("close\n");

  // TODO: remove from fd dictionary
  return 0;
}

void cmd_read_sd() {
  int file = 0;
  
  printf("sd test\n");
  // our FPGA hardware auto inits with ~100 cycles before 1st cmd after reset!
  /*
  *spimmc = 0x40000000; // 6 byte cmd0 + CRC, START(9) & STOP(1) bit!
  *spimmc16 = 0x0095;*/
  
  // send cmd0
  uint8_t status;
  uint8_t cmd0[6] = {0};
  uint8_t data[4];
  uint8_t cmd8[6] = {8, 0x00, 0x00, 0x01/*volt*/, 0xec/*check pat*/};
  uint8_t cmd16[6] = {16, 0x00, 0x00, 0x02, 0x00}; // 512 bytes block
  uint8_t acmd41[6] = {41, 0x40}; // SDHC or SDXC Supported
  
  for (int i = 0; i < 8; ++i) {
    status  = sd_cmd(cmd0, sizeof(cmd0));
    if (status == 0xff) {
      printf("no sd card\n");
      goto end;
    }
  }
  
  // in idle, SD only accepts CMD0, CMD1, CMD8, ACMD41, CMD58 and CMD59
  // SDC v2 check voltage ranges
  status = sd_cmd(cmd8, sizeof(cmd8), data, sizeof(data));
  if (status != 0x01) {
    printf("no cmd8 volt range\n");
  }
  
  // init & wait idle
  for (int i = 0; i < 64; ++i) {
    status = sd_acmd(acmd41, sizeof(acmd41), 0, 0);
    if (status == 0) {
      break;
    }
    if (status != 0x01 && status != 0x00) {
      printf("acmd error: %x\n", status);

#if 0
    // send init cmd1, for old cards only & wait idle
    for (int i = 0; i < 64; ++i) {
      uint8_t cmd1[6] = {1};
      status = sd_cmd(cmd1, sizeof(cmd1));
      if (status == 0)
	goto inited;
    }
    
    printf("sd card did not init\n");
    goto end;

#endif

      goto end;
    }
  }

 inited:
  
  printf("init'ed!\n");

  // re-set block size to 512, new cards might default to more
  status = sd_cmd(cmd16, sizeof(cmd16), 0, 0);
  if (status) {
    printf("set block size failed!\n");
  }

  status = sd_read(0, block, 512);
  if (status) goto end;
  
  // decode partition table
  uint32_t pstart;
  for (int i = 0; i < 4; ++i) {
    MbrPart* part = (MbrPart*)&block[446 + i * sizeof(MbrPart)];
    printf("%d: %d %d\n", i, part->first_lba, part->sectors);
    //printf("test\n"); //(int)part->type, (int)part->status);
    if (i == 0) pstart = part->first_lba;
  }
  fat.partlba = pstart;
  
  // decode FAT
  status = sd_read(fat.partlba, block, 512);
  if (status) goto end;
  
  {
    FatBootSector* bootsect = (FatBootSector*)block;
    printf("fat: sector: %d cluster size: %d fats: %d reserved: %d\n",
	   bootsect->bpb.bytesPerSector,
	   bootsect->bpb.sectorsPerCluster,
	   bootsect->bpb.fats,
	   bootsect->bpb.reservedSectors);
    fat.fat = bootsect->bpb.reservedSectors;
    fat.clusterSize = bootsect->bpb.sectorsPerCluster; // in sectors
    printf("fat: root: %d, fat sectors: %d %d\n",
	   bootsect->bpb.rootCluster,
	   bootsect->bpb.sectorsPerFAT,
	   bootsect->bpb.sectorsPerFAT2);
    fat.dataStart = bootsect->bpb.sectorsPerFAT2 * bootsect->bpb.fats;
    fat.rootCluster = bootsect->bpb.rootCluster;
  }

  file = open("MONKEY.TGA");
  if (file) {
    printf("TGA file: %d\n", file);
    
    size_t ret = 0;
    int header = 0;
    int x = 0, y = 199; // read bottom-up :-/
    int p = 0;

    volatile uint32_t* vga_ctrl = (volatile uint32_t*)0x808ffffc;
    volatile uint32_t* vga_pal =  (volatile uint32_t*)0x80810000;
    
    *vga_ctrl = 1;
#if 1
    // test gray palette
    for (int i = 0; i < 256; ++i) {
      //printf("pal: %d %x\n", i, vga_pal[i]);
      vga_pal[i] = (i << 16) | (i << 8) | (i << 0);
    }
#endif
    
    while (ret >= 0) {
      printf(".");
      
      ret = _read(file, block, sizeof(block));
      if (ret >= 0) {
	int j = 0;
	if (header == 0) {
	  // skip 18, rest palette
	  for (j = 18; j < 512; j += 3) {
	    vga_pal[p++] = (block[j+2] << 16) | (block[j+1] << 8) | (block[j+0] << 0);
	  }
	} else if (header == 1) {
	  // 768 - 512 - 18 still palette
	  for (j = 1; j < 274; j += 3) {
	    vga_pal[p++] = (block[j+3] << 16) | (block[j+1] << 8) | (block[j+0] << 0);
	  }
	}
	
	// any remaining pixel data until EOF trailer
	for (; y >= 0 && j < 512; j += 2) {
	  uint16_t v = *((uint16_t*)&block[j]);
	  vga_vram[y * 160 + x++] = v;
	  if (x >= 160) {
	    --y;
	    x = 0;
	  }
	}
	++header;
      }
    }
    
    printf("EOF\n");
    close(file); file = 0;
  }

  goto end;

  file = open("SONG61.WAV");
  if (file) {
    printf("WAV file: %d\n", file);
    
    uint8_t* highmem = (uint8_t*)sdram;
    highmemsize = 0;
    size_t ret = 0;
    while (ret >= 0) {
      printf(".");
      
      ret = _read(file, highmem, sizeof(block));
      if (ret >= 0) {
	highmem += sizeof(block);
	highmemsize += sizeof(block);
      }
    }
    printf("EOF\n");
    close(file); file = 0;
  }
  
 end:
  *spimmcOOB = 0xff; // OOB de-assert CS
}

int readfile(const char* filename, void* buf, size_t count)
{
  int fd = open(filename);
  if (fd <= 0) return fd;
  
  printf("readfile: %d\n", filename);
  int ret = read(fd, buf, count);
  
  close(fd);
  
  return ret;
}

void cmd_run()
{
  int ret = readfile("user.exe", sdram, 0x20000);
  if (ret <= 0) {
    printf("no user.exe\n");
    return;
  }
  
  printf("read: %d\n", ret);
  
  int (*entry)(int argc, char** argv) = (int (*)(int argc, char** argv))sdram;
  ret = entry(0, 0);
  printf("user.exe ret: %x\n", ret);
}


// --------------------------------------------------------

const uint8_t charset[] = {
  // custom charset, if you want any
};

// Conway's Game of Life

const uint8_t life_w = 40, life_h = 30;

struct pair {
  uint8_t x, y;
};

pair gosper_glider_gun[] = {
  {1,5}, {2,5}, {1,6}, {2,6},
  {35,3}, {36,3}, {35,4}, {36,4},
  
  {13,3}, {14,3},
  {12,4}, {16,4},
  {11,5}, {17,5},
  {11,6}, {15,6}, {17,6}, {18,6},
  {11,7}, {17,7},
  {12,8}, {16,8},
  {13,9}, {14,9},

  {25,1},
  {23,2}, {25,2},
  {21,3}, {22,3},
  {21,4}, {22,4},
  {21,5}, {22,5},
  {23,6}, {25,6},
  {25,7},
};

void life_set(uint8_t* state, int x, int y, bool v = true) {
  if (x < 0 || y < 0 || x >= life_w || y >= life_h)
    return;
  state[y * life_w + x] = v;
}

void life_set_array(uint8_t* state, pair* array, int n) {
  for (int i = 0; i < n; ++i) {
    life_set(state, array[i].x, array[i].y);
  }
}

__attribute__ ((__always_inline__))
uint8_t life_at(uint8_t* state, int x, int y) {
  if (x < 0 || y < 0 || x >= life_w || y >= life_h)
    return 0;
  return state[y * life_w + x] ? 1 : 0;
}

__attribute__ ((__always_inline__))
void life_draw(uint8_t* state) {
  uint8_t c = 1;
  for (int y = 0; y < life_h; ++y) {
    for (int x = 0; x < life_w; ++x, ++state) {
      // double text char as square pixels
      uint16_t v = *state > 0 ? (c++ << 12) : 0;
      // 32-bit at a time for performance
      vga_vram32[y*128/2 + x] = (v << 16) | v;
      if (c > 7) c = 1;
    }
  }
}

void life_block(uint8_t* state, int x, int y)
{
  life_set(state, x, y);
  life_set(state, x+1, y);
  life_set(state, x, y+1);
  life_set(state, x+1, y+1);
}

void life_blinker(uint8_t* state, int x, int y)
{
  life_set(state, x, y);
  life_set(state, x, y+1);
  life_set(state, x, y+2);
}

void life_glider(uint8_t* state, int x, int y)
{
  life_set(state, x+1, y);
  life_set(state, x+2, y+1);
  life_set(state, x, y+2);
  life_set(state, x+1, y+2);
  life_set(state, x+2, y+2);
}

void life_seed(uint8_t* state) {
  memset(state, 0, life_w * life_h);
  
  //life_block(state, 1, 1);
  //life_glider(state, 4, 4);
  //life_blinker(state, life_w/2, life_h/2);
  
#if 0
  life_set(state, life_w/2 - 1, life_h/2 - 1);
  life_set(state, life_w/2 - 1, life_h/2 + 1);
  life_set(state, life_w/2 - 2, life_h/2 - 1);
  life_set(state, life_w/2 - 2, life_h/2 + 1);
  life_set(state, life_w/2 - 3, life_h/2 - 1);
  life_set(state, life_w/2 - 3, life_h/2 + 0);
  life_set(state, life_w/2 - 3, life_h/2 + 1);

  life_set(state, life_w/2 + 1, life_h/2 - 1);
  life_set(state, life_w/2 + 1, life_h/2 + 1);
  life_set(state, life_w/2 + 2, life_h/2 - 1);
  life_set(state, life_w/2 + 2, life_h/2 + 1);
  life_set(state, life_w/2 + 3, life_h/2 - 1);
  life_set(state, life_w/2 + 3, life_h/2 + 0);
  life_set(state, life_w/2 + 3, life_h/2 + 1);
#else
  life_set_array(state, gosper_glider_gun,
		 sizeof(gosper_glider_gun) / sizeof(*gosper_glider_gun));
#endif
}

__attribute__ ((__always_inline__))
void life_evolve(uint8_t* state, uint8_t* state2) {
  for (int y = 0; y < life_h; ++y) {
    for (int x = 0; x < life_w; ++x) {
      // surrounding popcount
      uint8_t count =
	life_at(state, x-1, y-1) +
	life_at(state, x,   y-1) +
	life_at(state, x+1, y-1) +
	life_at(state, x-1, y) +
	life_at(state, x+1, y) +
	life_at(state, x-1, y+1) +
	life_at(state, x,   y+1) +
	life_at(state, x+1, y+1);
      
      // dead or alive?
      *state2++ = (count == 3) || (count == 2 && state[y * life_w + x]);
    }
  }
}

//uint8_t state[life_w * life_h * 2];

void life_init() {
  uint8_t* state = (uint8_t*)sdram;
  uint8_t* state2 = state + life_w * life_h;

  life_seed(state);
}

__attribute__((noinline, section(".fastcode")))
void life_run(int bench) {
  uint8_t* state = (uint8_t*)sdram;
  uint8_t* state2 = state + life_w * life_h;

  uint32_t cycles_begin, cycles_end;
  uint32_t instns_begin, instns_end;
  RDCYCLE(cycles_begin);
  RDINSTR(instns_begin);

  life_draw(state);
  for (int i = 0; i < (bench ? 32 : 666667); ++i) {
    life_evolve(state, state2);
    life_draw(state2);
    life_evolve(state2, state);
    life_draw(state);
    if (reg_uart_data != -1)
      break;
  }

  if (bench) {
    printf("Cycles: 0x%x\n", cycles_end - cycles_begin);
    printf("Instns: 0x%x\n", instns_end - instns_begin);
  }
}

const char banner[80] = "RX32 SoC, initializing. https://rene.rene.de ;-)";

const uint32_t jit[] = {
  0x00400593,
  0x00b55533,
  0x00050533,
  0x00008067,
};

const uint16_t fbimg[] = {
};

extern "C"
void main()
{
        volatile uint32_t* vga_mmio = (volatile uint32_t*)0x80800000;
	uint32_t cursor = 0;
	
	reg_leds = 31;
	reg_uart_clkdiv = SYSCLK / 115000;
	
	print("Booting..\n");

	reg_leds = 63;
	set_flash_qspi_flag();
	
	for (int i = 0; i < sizeof(charset); ++i)
	  vga_font[i] = charset[i];
	
	reg_leds = 127;
	while (getchar_prompt("Press ENTER to continue..\n") != '\r') { /* wait */ }
	
	print(" ___ __   __ __   ___\n");
	print("| _ \\\\ \\_/ /|__`.(_  |\n");
	print("| v / > , <  |_ | / / \n");
	print("|_|_\\/_/ \\_\\|__.'|___|\n");
	printf(" @%dMHz MEM: %dKiB\n\n", SYSCLK / 1000000, MEM_TOTAL / 1024);

	//cmd_memtest(0); print("\n");
	
	set_flash_mode_qddr(); // default to qddr for our convinience
	
	cmd_print_spi_state();
	print("\n");
	
	while (1)
	{
		print("\n");

		print("Select an action:\n");
		print("[1] Read SPI Flash ID       [2] Read SPI Config Regs\n");
		print("[3] Switch to default mode  [4] Switch to Dual I/O mode\n");
		print("[5] Switch to Quad I/O mode [6] Switch to Quad DDR mode\n");
		print("[7] Toggle continuous read mode [9] Run simplistic benchmark\n");
		print("[0] Benchmark all configs   [M] Run Memtest\n");
		print("[S] Print SPI state         [c] Move VGA cursor [d] Dac echo UART\n");
		print("[e] Echo UART               [g] graphic mode test\n");

		for (int rep = 10; rep > 0; rep--)
		{
			print("Cmd> ");
			char cmd = getchar();
			if (cmd > 32 && cmd < 127)
				putchar(cmd);
			print("\n");

			switch (cmd)
			{
			case 'j':
			        {
				  print("jit> ");
				  int (*x)(int a) = (int (*)(int a))jit;
				  int i = x(0xc0febeed);
				  printf("%x\n", i);
				}
			        break;
			case '1':
				cmd_read_flash_id();
				break;
			case '2':
				cmd_read_flash_regs();
				break;
			case '3':
				set_flash_mode_spi();
				break;
			case '4':
				set_flash_mode_dual();
				break;
			case '5':
				set_flash_mode_quad();
				break;
			case '6':
				set_flash_mode_qddr();
				break;
			case '7':
				reg_spictrl = reg_spictrl ^ 0x00100000;
				break;
			case '9':
				cmd_benchmark(true, 0);
				break;
			case '0':
				cmd_benchmark_all();
				break;
			case 'm':
			case 'M':
				cmd_memtest(cmd == 'M');
				break;
			case 'S':
				cmd_print_spi_state();
				break;
			case 'e':
				cmd_echo();
				break;
			case 'a':
			        cmd_analyze();
				break;
			case 'A':
			        cmd_midi();
				break;
			case 'r':
			        cmd_read_ram();
				break;
			case 'R':
			        cmd_run();
				break;
			case 's':
			        cmd_read_sd();
				break;
			case 'd':
			case 'D':
				cmd_dac(cmd == 'D');
				break;
			case ' ':
			        scroll ^= 2; // or 1
				break;
			case 'l':
			case 'L':
			        life_init();
				life_run(cmd == 'L');
			        break;
			case 'g':
			case 'G':
			        {
				  volatile uint32_t* vga_ctrl = (volatile uint32_t*)0x808ffffc;
				  *vga_ctrl = (*vga_ctrl & 1) ^ 1;
				  if (cmd == 'g') {
				    for (int y = 0; y < 240; ++y)
				      vga_vram[y*320/8/2] = 0b1110001000000000;
				    //vga_vram8[y*320/8 * 4 + 3] = 0b11100010;
				  } else {
				    for (int i = 0; i < sizeof(fbimg) / sizeof(*fbimg); ++i)
				      vga_vram[i] = fbimg[i];
				    while (getchar_prompt(">") != '\r') { /* wait */ }
				  }
				}
				break;
			case 'c':
			case 'C':
			  {
			    for (int y = 0; y < 30; ++y)
			      for (int x = 0; x < 80; ++x)
				vga_vram[y*128 + x] = cursor << 12;
			    
			    for (int y = cursor; y < cursor + 1; ++y) {
			      for (int x = cursor, i = 0; x < 80 - cursor; ++x, ++i) {
				uint8_t attr = ((y & 0xf) << 4) | (x & 0xf);
				vga_vram[y*128 + x] = (attr << 8) | banner[i];
			      }
			    }
			    
#if 1
				  cursor += cmd == 'c' ? 1 : -1;
				  printf("Cursor: %04d, _%2d_\n", vga_mmio[0], vga_mmio[1]);
				  vga_mmio[0] = cursor * 4;
				  vga_mmio[1] = cursor * 4;
				  uint32_t cycles;
				  RDCYCLE(cycles);
				  vga_mmio[2] = cycles;
				  vga_mmio[3] = ~cycles;
#else
				  if (cmd == 'c') {
				    print("VGA: %x %x %x %x\n",
					  vga_vram[0], vga_vram[1], vga_vram[2], vga_vram[4]);
				  } else {
				    uint32_t cycles;
				    RDCYCLE(cycles_now);
				    vga_vram[0] = cycles;
				    vga_vram[1] = cycles >> 16;
				    RDCYCLE(cycles);
				    vga_vram[2] = cycles;
				    vga_vram[3] = cycles >> 16;
				  }
#endif
				}
				break;
			default:
				continue;
			}

			break;
		}
	}
}
