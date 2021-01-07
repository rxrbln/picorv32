/*
 *  PicoSoC - A simple example user program using PicoRV32
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
  
__attribute__((section(".fastcode"))) void* memset(void*, int, size_t);

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

static uint32_t vgax = 0, vgay = 0;

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

int getone() {
  // serial
  int c = reg_uart_data;
  
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
  
  return c;
}

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

uint8_t life_at(uint8_t* state, int x, int y) {
  if (x < 0 || y < 0 || x >= life_w || y >= life_h)
    return 0;
  return state[y * life_w + x] ? 1 : 0;
}

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

uint8_t state[life_w * life_h * 2];

__attribute__((section(".start")))
int main (int argc, char** argv) {
  //uint8_t* state = (uint8_t*)sdram;
  uint8_t* state2 = state + life_w * life_h;
  const uint8_t bench = argc > 1;

  uint32_t cycles_begin, cycles_end;
  uint32_t instns_begin, instns_end;
  if (bench) {
    RDCYCLE(cycles_begin);
    RDINSTR(instns_begin);
  }
  
  life_seed(state);
  life_draw(state);
  for (int i = 0; i < (bench ? 32 : 666667); ++i) {
    life_evolve(state, state2);
    life_draw(state2);
    life_evolve(state2, state);
    life_draw(state);
    if (getone() != -1)
      break;
  }

  if (bench) {
    printf("Cycles: 0x%x\n", cycles_end - cycles_begin);
    printf("Instns: 0x%x\n", instns_end - instns_begin);
  }
  
  return 0;
}
