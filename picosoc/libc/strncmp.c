/* Copyright (c) 2020 ExactCODE GmbH. All rights reserved. */

int strncmp(const char *s1, const char *s2, size_t n) {
  const unsigned char* a = (const unsigned char*)s1;
  const unsigned char* b = (const unsigned char*)s2;
  for (const unsigned char* fini = a + n; a != fini; ++a, ++b) {
    int res = *a - *b;
    if (res) return res;
    if (!*a) return 0;
  }
  return 0;
}
