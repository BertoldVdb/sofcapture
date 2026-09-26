/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* User API of the sofclock module; mirrored in sofd's src/kmod.rs. */
#ifndef SOFCLOCK_H
#define SOFCLOCK_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* xHCI MFINDEX, 125 µs microframes */
#define SOFCLOCK_MODE_XHCI_PCI	0
/* any controller; step size and wrap point depend on the driver */
#define SOFCLOCK_MODE_GENERIC	1

struct sofclock_req {
	__u32 mode;
	/* in, XHCI_PCI: PCI address of a controller bound to xhci_hcd */
	__u32 domain;
	__u8 bus;
	__u8 devfn;
	/* in: steps to time, 1..4; the tightest is returned */
	__u16 edges;
	/* in, GENERIC: USB bus number of a root hub */
	__u32 busnum;
	/* in: max time with interrupts off while spinning; 0 = never */
	__u32 irq_off_max_ns;
	/* out: counter value after the step */
	__u32 counter;
	/* out: tick length; 0 = unknown (GENERIC) */
	__u32 unit_ns;
	__u32 reserved;
	/* out: CLOCK_MONOTONIC_RAW time of the step, and half its bracket width */
	__s64 t_ns;
	__s64 unc_ns;
};

/*
 * Errors: ENODEV no controller or driver, EOPNOTSUPP not xHCI, EIO counter
 * not running, EAGAIN no clean step (retry).
 */
#define SOFCLOCK_IOC_SAMPLE _IOWR('S', 0x03, struct sofclock_req)

#endif
