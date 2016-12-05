// Copyright (c) Formula Slug 2016. All Rights Reserved.

#pragma once

#include <cstdio>

#ifdef __cplusplus
namespace std {
extern "C" void* memcpy(void* dst, const void* src, size_t count);
}

#include <stdlib.h>

void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void* ptr);
void operator delete(void* ptr, size_t size);
void operator delete[](void* ptr);
void operator delete[](void* ptr, size_t size);

__extension__ typedef int __guard __attribute__((mode(__DI__)));

extern "C" int __cxa_guard_acquire(__guard*);
extern "C" void __cxa_guard_release(__guard*);
extern "C" void __cxa_guard_abort(__guard*);
extern "C" void __cxa_pure_virtual(void);
#else
extern void* memcpy(void* dst, const void* src, size_t count);
#endif  // __cplusplus
