#ifndef __CTYPES_H
#define __CTYPES_H

/* Copyright (c) 2018-2020 ExactCODE GmbH. All rights reserved. */

int isdigit(int c) {
  return (c >= '0' && c <= '9') ? c : 0;
}

int tolower(int c) {
  return (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c;
}

int toupper(int c) {
  return (c >= 'a' && c <= 'z') ? c - ('a' - 'A') : c;
}

#endif
