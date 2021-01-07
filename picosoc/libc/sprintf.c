
int sprintf(char* text, const char* format, ...)
  __attribute__ ((format (printf, 2, 3)));

int vsprintf(char* out, const char* format, va_list argp)
  __attribute__ ((format (printf, 2, 0)));
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
#ifdef WANT_PRINTF_FLOAT
      } else if (fmt == 'f')  {
        float float_to_print = va_arg(argp, double);
	// format float - TODO: all the really complex stuff, ...
	int _ = float_to_print;
	unsigned __ = (float_to_print * 10);
	out += sprintf(out, "%d.%d", _, __ % 10);
#endif
      } else {
        out += sprintf(out, "NYI: %%%c", fmt);
      }
    } else {
#ifdef DOS
      // TODO: only for non-binary streams, sigh!
      if (*format == '\n')
	*out++ = '\r';
#endif
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
{
  va_list argp, argp2;
  va_start(argp, format);
  va_copy(argp2, argp);
  int ret = vsprintf(text, format, argp2);
  va_end(argp2);
  
  return ret;
}
