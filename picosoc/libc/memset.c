/* Copyright (c) 2010-2020 ExactCODE GmbH. All rights reserved. */

void* memset(void* b, int c, size_t len)
{
  uint8_t* ptr = (uint8_t*)b;

  //for (; len > 0; --len, ptr++)  *ptr = c;

  // optimize to int sized stores
  for (; len >= sizeof(int); len -= sizeof(int), ptr += sizeof(int))
    *((int*)ptr) = (c << 24) | (c << 16) | (c << 8) | c; // NYI: larger int
  
  // reminder a byte at a time
  for (; len > 0; --len, ptr++)
    *ptr = c;
  return b;
}
