#pragma once

// GCC spellings from Zephyr's toolchain headers.
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif
#ifndef __aligned
#define __aligned(x) __attribute__((__aligned__(x)))
#endif
#ifndef __weak
#define __weak __attribute__((__weak__))
#endif

// Without user mode, Zephyr syscalls are inline wrappers (see syscalls/can.h).
#define __syscall static inline
#define __subsystem

#define BUILD_ASSERT(EXPR, MSG...) _Static_assert((EXPR), "" MSG)
#define ARG_UNUSED(x) (void)(x)
