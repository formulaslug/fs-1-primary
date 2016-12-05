// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "StdLib.h"

#include <cstdio>

#include "ch.h"

namespace std {
void __throw_bad_alloc() { printf("Unable to allocate memory"); }

void __throw_length_error(const char* e) {
  printf("Length Error :%s\n", e);
}
}

void* operator new(size_t size) {
  return chHeapAllocAligned(nullptr, size, 0);
}

void* operator new[](size_t size) {
  return chHeapAllocAligned(nullptr, size, 0);
}

void operator delete(void* ptr) {
  chHeapFree(ptr);
}

void operator delete(void* ptr, size_t size) {
  static_cast<void>(size);

  chHeapFree(ptr);
}

void operator delete[](void* ptr) {
  free(ptr);
}

void operator delete[](void* ptr, size_t size) {
  static_cast<void>(size);

  chHeapFree(ptr);
}

//int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
//void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
//void __cxa_guard_abort (__guard *) {};

//void __cxa_pure_virtual(void) {};
