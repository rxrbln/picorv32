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

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

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
#define reg_dac ((volatile uint32_t*)0x40000000)
#define reg_fm ((volatile uint32_t*)0x40000010)
#define reg_fm16 ((volatile uint16_t*)0x40000010)

static uint32_t* vga_vram = (uint32_t*)0x80000000;
static uint32_t* vga_font = (uint32_t*)0x80002000;

static volatile uint32_t* sdram = (uint32_t*)0x20000000;

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
	uint8_t buffer_wr[5] = {0x71, addr >> 16, addr >> 8, addr, 0x70 | value};
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

uint8_t scroll = 0;

void putchar(char c)
{
  if (c == '\n')
    reg_uart_data = '\r';
 
#if 1
  vga_vram[0] = 0x6200 | 'R';
  vga_vram[1] = 0x1f00 | 'X';
  vga_vram[2] = 0x2900 | '3';
  vga_vram[3] = 0x8500 | '\xfd';

  if (c != '\n' && c != '\r')
  vga_vram[vgay * 128 + vgax++] =
    //(c == 'E' ? 0xe200 : 0x700)
    //(vgax + 1) << 8
    0x700 // color attribute: white on black
    | c;
  
  if (c == '\n') {
    vgax = 0;
    ++vgay;
    if (vgay >= 30){
      if (!scroll) {
	vgay = 0; // simply wrap
      } else {
	// scroll
	for (vgay = 0; vgay < 30 - scroll; ++vgay)
	  for (int x = 0; x < 80; ++x)
	    vga_vram[vgay * 128 + x] = vga_vram[(vgay + scroll) * 128 + x];
	// clear lines after curret, only for scroll > 1
	for (int y = vgay + 1; y < 30; ++y)
	  for (int x = 0; x < 80; ++x)
	    vga_vram[y * 128 + x] = 0;
      }
    }
    
    // clear last, new line
    for (int x = 0; x < 80; ++x)
      vga_vram[vgay * 128 + x] = 0;
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

int isdigit(int c) {
  if (c >= '0' && c <= '9')
    return c;
  return 0;
}

#ifndef __riscv_muldiv

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

#endif

int sprintf(char* out, const char* format, ...)
  __attribute__ ((format (printf, 2, 3)));
int vsprintf(char* out, const char* format, va_list argp)
{
  const char* _out = out;
  char temp[23]; // temp. formating buffer
  uint8_t tempi = sizeof(temp);
  for (; *format; ++format) {
    // TODO: >9 (single digit) format and precision!
    // TODO: and support for all formats, ...
    uint8_t precision = 0, width = 0, zeros = 0;
    
    if (*format == '%') {
      ++format;
      if (*format == '0') {
	zeros = *format++;
      }
      if (isdigit(*format)) {
	width = *format++ - '0';
      }
      if (format[0] == '.' && isdigit(format[1])) {
	precision = format[1] - '0';
	format += 2;
      }
      const char fmt = *format;
      if (fmt == '%') {
        *out++ = '%';
      } else if (fmt == 'c') {
        char char_to_print = va_arg(argp, int);
        *out++ = char_to_print;
      } else if (fmt == 's') {
        char* str_to_print = va_arg(argp, char*);
        while (*str_to_print) {
	  *out++ = *str_to_print++;
	  if (precision && --precision == 0) break;
	}
      } else if (fmt == 'd' || fmt == 'i')  {
	int int_to_print = va_arg(argp, int);
	if (int_to_print < 0) {
	  *out++ = '-';
	  int_to_print = -int_to_print;
	}
	// format int
	do {
	  int _ = int_to_print % 10;
	  temp[--tempi] = '0' + _;
	  int_to_print /= 10;
	} while (int_to_print);
	goto output;
      } else if (fmt == 'u')  {
	unsigned int_to_print = va_arg(argp, unsigned);
	// format unsigned
	do {
	  int _ = int_to_print % 10;
	  temp[--tempi] = '0' + _;
	  int_to_print /= 10;
	} while (int_to_print);
	goto output;
      } else if (fmt == 'x' || fmt == 'p' || fmt == 'X') {
	unsigned hex_to_print = va_arg(argp, int);
	// hex format int
	do {
	  int _ = hex_to_print & 0xf;
	  if (_ > 9) _ += (fmt == 'X' ? 'A' : 'a') - '9' - 1;
	  temp[--tempi] = '0' + _;
	  hex_to_print >>= 4;
	} while (hex_to_print);
	goto output;
#if 0
      } else if (fmt == 'f')  {
        float float_to_print = va_arg(argp, double);
	// format float - TODO: all the really complex stuff, ...
	int _ = float_to_print;
	unsigned __ = (float_to_print * 10);
	out += sprintf(out, "%d.%d", _, __ % 10);
#endif
      } else {
        out += sprintf(out, "NIY: %%%c", fmt);
      }
    } else {
      *out++ = *format;
    }

    continue;
  
  output:
    // copy to output buffer and implicitly reset tempi
    if (width) {
      const int w = sizeof(temp) - tempi;
      for (int i = 0; i < width - w; ++i)
	*out++ = zeros ? '0' : ' ';
    }
    for (; tempi < sizeof(temp); ++tempi)
      *out++ = temp[tempi];
  }

  *out = 0;
  return out - _out;
}

int sprintf(char* text, const char* format, ...)
  __attribute__ ((format (printf, 2, 3)));
int sprintf(char* text, const char* format, ...)
{
  va_list argp, argp2;
  va_start(argp, format);
  va_copy(argp2, argp);
  int ret = vsprintf(text, format, argp2);
  va_end(argp2);
  
  return ret;
}

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

char getchar_prompt(char *prompt)
{
	int32_t c = -1;
	uint32_t cycles_begin, cycles_now, cycles;
	__asm__ volatile ("rdcycle %0" : "=r"(cycles_begin));

	reg_leds = ~0;

	if (prompt)
		print(prompt);

	while (c == -1) {
		__asm__ volatile ("rdcycle %0" : "=r"(cycles_now));
		cycles = cycles_now - cycles_begin;
		if (cycles > SYSCLK / 2) {
			if (prompt)
				print(prompt);
			cycles_begin = cycles_now;
			reg_leds = ~reg_leds;
		}
		c = reg_uart_data;
	}

	reg_leds = 0;
	return c;
}

char getchar()
{
	return getchar_prompt(0);
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

	volatile uint32_t *base_word = (uint32_t *) (ext ? sdram : 0);
	volatile uint8_t *base_byte = (uint8_t *) (ext ? sdram : 0);
	const int mem_total = ext ? 64*1024*1024 : MEM_TOTAL;
	print("Running memtest ");

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

	uint8_t buffer[6] = {0x65, addr >> 16, addr >> 8, addr, 0, 0};
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
	__asm__ volatile ("rdcycle %0" : "=r"(cycles_begin));
	__asm__ volatile ("rdinstret %0" : "=r"(instns_begin));

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

	__asm__ volatile ("rdcycle %0" : "=r"(cycles_end));
	__asm__ volatile ("rdinstret %0" : "=r"(instns_end));

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

const uint16_t audiofile[] = {
};

const uint8_t vgmfile[] = {
};

uint8_t getbyte() {
  int32_t c = -1;
  while (c == -1) {
    c = reg_uart_data;
  }
  return c;
}

#define MAX(a,b) ((a) > (b) ? (a) : (b))


const uint16_t sn_vol_tab[16] = {
  32767, 26028, 20675, 16422, 13045, 10362,  8231,  6568,
  5193,  4125,  3277,  2603,  2067,  1642,  1304,     0
};


void cmd_dac(uint8_t alt)
{
  print("DAC & FM/VGM testing\n");
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
  
  
  for (int i = 0; i < sizeof(audiofile) / sizeof(*audiofile);) {
    reg_dac[0] = audiofile[i++]; // left
    reg_dac[1] = audiofile[i++]; // right
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
	__asm__ volatile ("rdcycle %0" : "=r"(cycles_now));
      } while (cycles_now - cycles_begin < wait);
    }
    // update after reach command to help compensate expensive div delay?
    __asm__ volatile ("rdcycle %0" : "=r"(cycles_begin));
  }
  
 return_silence:
  reg_dac[0] = 0; // silence DAC

  // disable all FM ops that might be left running:
  for (uint8_t i = 0; i < 4; ++i)
    reg_fm[i] = 0;
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

uint8_t sd_status() {
  uint8_t status;
  // wait for status
  printf("status: ");
  for (int i = 0; i < 16; ++i) {
    status = *spimmc;
      printf(" %x", status);
      if (status != 0xff)
	break;
  }
  if (status) {
    printf(":", status);
    printf("%s%s%s%s%s%s%s\n",
	   status & BIT(6) ? " ParameterError" : "",
	   status & BIT(5) ? " AddressError" : "",
	   status & BIT(4) ? " EraseSeqError" : "",
	   status & BIT(3) ? " CommandCRCReror" : "",
	   status & BIT(2) ? " IllegalCommand" : "",
	   status & BIT(1) ? " EraseReset" : "",
	   status & BIT(0) ? " Idle" : "");
  }
  
  return status;
}

uint8_t sd_cmd(uint8_t* cmd, int size, uint8_t* data = 0, int dsize = 0) {
  *spimmcOOB = 0xff; // OOB de-assert CS, start new command
  
  cmd[0] |= 0x40;
  cmd[size - 1] = crc7(cmd, size - 1) | 1; // stop bit
  for (int i = 0; i < size; ++i) {
    *spimmc8 = cmd[i];
    printf("%02x", cmd[i]);
  }
  printf(" - ");
  uint8_t status = sd_status();
  // response payload
  if (dsize > 0 && (status == 0 || status == 1)) {
    for (int i = 0; i < dsize; ++i) {
      data[i] = *spimmc;
      printf("%02x", data[i]);
    }
    printf("\n");
  }
  
  return status;
}

uint8_t sd_acmd(uint8_t* acmd, int size, uint8_t* data, int dsize) {
  // 1st send CMD55 ACMD prefix command
  uint8_t cmd55[6] = {55};
  uint8_t status = sd_cmd(cmd55, sizeof(cmd55));
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
  uint8_t cmd17[6] = {17, addr >> 24, addr >> 16, addr >> 8, addr};
  uint8_t status = sd_cmd(cmd17, sizeof(cmd17));
  
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
    for (int i = 0; i < dsize; ++i) {
      data[i] = *spimmc;
      printf("%02x", data[i]);
    }
    printf("\n");
  }

  uint16_t crc = (*spimmc) << 16 | *spimmc;
  printf("crc: %04x\n", crc);

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
  
  struct Partition {
    uint8_t status;
    uint8_t first_chs[3];
    uint8_t type;
    uint8_t last_chs[3];
    uint32_t first_lba;
    uint32_t sectors;
  };


void cmd_read_sd() {
  printf("sd test\n");
  // our FPGA hardware auto inits with ~100 cycles before 1st cmd after reset!
  /*
  *spimmc = 0x40000000; // 6 byte cmd0 + CRC, START(9) & STOP(1) bit!
  *spimmc16 = 0x0095;*/
  
  // send cmd0
  uint8_t cmd0[6] = {0};
  uint8_t data[4];
  uint8_t cmd8[6] = {8, 0x00, 0x00, 0x01/*volt*/, 0xec/*check pat*/};
  uint8_t acmd41[6] = {41, 0x40};
  uint8_t block[512];
   
  uint8_t status = sd_cmd(cmd0, sizeof(cmd0));
  if (status == 0xff) {
    printf("no sd card\n");
    goto end;
  }

  // in idle, SD only accepts CMD0, CMD1, CMD8, ACMD41, CMD58 and CMD59
  // SDC v2 check voltage ranges
  status = sd_cmd(cmd8, sizeof(cmd8), data, sizeof(data));
  if (status != 0x01) {
    printf("no cmd8 volt range\n");
    
    // send init cmd1, for old cards only
    uint8_t cmd1[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = sd_cmd(cmd1, sizeof(cmd1));
    if (status != 0) {
      printf("sd card did not init\n");
      goto end;
    }
    goto inited;
  }
  
  // init & wait idle
  for (int i = 0; i < 64; ++i) {
    status = sd_acmd(acmd41, sizeof(acmd41), 0, 0);
    if (status == 0) {
      break;
    }
    if (status != 0x01 && status != 0x00) {
      printf("acmd error: %x\n", status);
      goto end;
    }
  }

 inited:
  
  printf("init'ed!\n");
  
  status = sd_read(0, block, sizeof(block));
  printf("read: %x\n", status);
  
  // decode partition table
  

 end:
  *spimmcOOB = 0xff; // OOB de-assert CS
}

// --------------------------------------------------------

const uint8_t charset[] = {
  // custom charset, if you want any
};

const char banner[80] = "RX32 SoC, initializing. https://rene.rene.de ;-)";

const uint32_t jit[] = {
  0x00400593,
  0x00b55533,
  0x00050533,
  0x00008067,
};

const uint16_t fbimg[] = {
};

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
			case 'r':
			        cmd_read_ram();
				break;
			case 's':
			        cmd_read_sd();
			        //scroll ^= 2; // or 1
				break;
			case 'd':
			case 'D':
				cmd_dac(cmd == 'D');
				break;
			case 'g':
			case 'G':
			        {
				  volatile uint32_t* vga_ctrl = (volatile uint32_t*)0x808ffffc;
				  *vga_ctrl ^= 1;
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
				  __asm__ volatile ("rdcycle %0" : "=r"(cycles));
				  vga_mmio[2] = cycles;
				  vga_mmio[3] = ~cycles;
#else
				  if (cmd == 'c') {
				    print("VGA: %x %x %x %x\n",
					  vga_vram[0], vga_vram[1], vga_vram[2], vga_vram[4]);
				  } else {
				    uint32_t cycles;
				    __asm__ volatile ("rdcycle %0" : "=r"(cycles));
				    vga_vram[0] = cycles;
				    vga_vram[1] = cycles >> 16;
				    __asm__ volatile ("rdcycle %0" : "=r"(cycles));
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
