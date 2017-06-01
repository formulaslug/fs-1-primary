// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

#ifdef __cplusplus

#include <stddef.h>

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
#endif  // __cplusplus

#ifdef __cplusplus
namespace std {
#endif  // __cplusplus
int printf(const char* format, ...);
#ifdef __cplusplus
}  // std
#endif  // __cplusplus
