// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "StdLib.h"

#include <cstdarg>
#include <cstdlib>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

namespace std {
void __throw_bad_alloc() { printf("Unable to allocate memory"); }

void __throw_length_error(const char* e) {
  printf("Length Error :%s\n", e);
}
}

void* operator new(size_t size) { return chHeapAllocAligned(nullptr, size, 1); }

void* operator new[](size_t size) {
  return chHeapAllocAligned(nullptr, size, 1);
}

void operator delete(void* ptr) { chHeapFree(ptr); }

void operator delete(void* ptr, size_t size) {
  static_cast<void>(size);

  chHeapFree(ptr);
}

void operator delete[](void* ptr) { chHeapFree(ptr); }

void operator delete[](void* ptr, size_t size) {
  static_cast<void>(size);

  chHeapFree(ptr);
}

// int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
// void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
// void __cxa_guard_abort (__guard *) {};

// void __cxa_pure_virtual(void) {};

namespace std {
int printf(const char* format, ...) {
  static BaseSequentialStream* chp =
      reinterpret_cast<BaseSequentialStream*>(&SD1);
  static bool init = false;
  if (!init) {
    // Activate serial driver 1 using the default driver configuration
    sdStart(&SD1, nullptr);
    init = true;
  }

  int size = 0;
  va_list ap;

  va_start(ap, format);
  size = chvprintf(chp, format, ap);
  va_end(ap);

  return size;
}
}  // std
