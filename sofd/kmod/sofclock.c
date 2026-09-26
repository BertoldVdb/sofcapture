// SPDX-License-Identifier: GPL-2.0
/*
 * sofclock: times USB host controller frame counter steps for sofd.
 *
 * XHCI_PCI reads MFINDEX through xhci-pci's own mapping, for kernels that
 * won't let user space map it (CONFIG_IO_STRICT_DEVMEM, lockdown).
 * GENERIC polls usb_get_current_frame_number() and works with any driver.
 */
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "sofclock.h"

#define MFINDEX_MASK	0x3fff
#define HS_UNIT_NS	125000
#define FRAME_NS	1000000
#define MAX_EDGES	4
#define MAX_IRQ_OFF_NS	500000
/*
 * A clean bracket is two reads long, but read times vary (PCIe: 0.5-1.5 µs),
 * so allow some slack. An interrupt costs several µs.
 */
#define CLEAN_READS	4
#define CLEAN_SLACK_NS	1000

struct counter_src {
	/* counter, or negative errno */
	int (*read)(struct counter_src *src);
	void __iomem *mfindex;
	struct usb_device *root_hub;
};

static int read_mfindex(struct counter_src *src)
{
	return readl(src->mfindex) & MFINDEX_MASK;
}

static int read_frame_number(struct counter_src *src)
{
	return usb_get_current_frame_number(src->root_hub);
}

/*
 * Spin until `edges` steps were seen or `budget_ns` passed. If `slice_ns` is
 * set, interrupts are off in slices of at most that long.
 */
static int time_steps(struct counter_src *src, u32 edges, u64 budget_ns, u64 slice_ns,
		      struct sofclock_req *r)
{
	unsigned long flags = 0;
	u64 t_prev, t_cur, t_next, deadline, slice_end, min_read;
	int v_start, v_prev, v;
	u32 found = 0;
	s64 best = S64_MAX;

	if (slice_ns)
		local_irq_save(flags);
	t_prev = ktime_get_raw_ns();
	v_start = v_prev = src->read(src);
	t_cur = ktime_get_raw_ns();
	min_read = t_cur - t_prev;
	deadline = t_prev + budget_ns;
	slice_end = t_prev + slice_ns;
	if (v_start < 0) {
		if (slice_ns)
			local_irq_restore(flags);
		return -EIO;
	}
	/*
	 * Order: t_prev, read(v_prev), t_cur, read(v), t_next. A step first
	 * seen in v happened within [t_prev, t_next]. Skip jumps by more than
	 * one and brackets stretched by an interrupt or stall.
	 */
	for (;;) {
		if (slice_ns && t_cur > slice_end) {
			local_irq_restore(flags);
			cpu_relax();
			local_irq_save(flags);
			slice_end = ktime_get_raw_ns() + slice_ns;
		}
		v = src->read(src);
		t_next = ktime_get_raw_ns();
		if (v < 0)
			break;
		if (t_next - t_cur < min_read)
			min_read = t_next - t_cur;
		if (v != v_prev &&
		    (v == v_prev + 1 ||
		     (src->mfindex ? v == ((v_prev + 1) & MFINDEX_MASK) : v == 0)) &&
		    t_next - t_prev <= CLEAN_READS * min_read + CLEAN_SLACK_NS) {
			s64 unc = (s64)((t_next - t_prev + 1) >> 1);

			if (unc < best) {
				best = unc;
				r->counter = v;
				r->t_ns = (s64)(t_prev + ((t_next - t_prev) >> 1));
				r->unc_ns = unc;
			}
			if (++found >= edges)
				break;
		}
		t_prev = t_cur;
		t_cur = t_next;
		v_prev = v;
		if (t_next > deadline)
			break;
	}
	if (slice_ns)
		local_irq_restore(flags);

	if (found)
		return 0;
	return (v < 0 || v == v_start) ? -EIO : -EAGAIN;
}

static int sample_xhci_pci(struct sofclock_req *r)
{
	struct counter_src src = { .read = read_mfindex };
	struct pci_dev *pdev;
	struct usb_hcd *hcd;
	u32 caplen, rtsoff;
	int ret;

	pdev = pci_get_domain_bus_and_slot(r->domain, r->bus, r->devfn);
	if (!pdev)
		return -ENODEV;
	if (pdev->class != PCI_CLASS_SERIAL_USB_XHCI) {
		ret = -EOPNOTSUPP;
		goto put_dev;
	}

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&pdev->dev);
		goto put_dev;
	}

	/* keeps xhci-pci and its mapping bound */
	device_lock(&pdev->dev);
	if (!pdev->dev.driver || strcmp(pdev->dev.driver->name, "xhci_hcd")) {
		ret = -ENODEV;
		goto unlock;
	}
	hcd = pci_get_drvdata(pdev);
	if (!hcd || !hcd->regs || hcd->rsrc_len < 0x20) {
		ret = -ENODEV;
		goto unlock;
	}
	caplen = readl(hcd->regs) & 0xff;
	rtsoff = readl(hcd->regs + 0x18) & ~0x1fu;
	if (caplen < 0x20 || rtsoff + 4 > hcd->rsrc_len || caplen + 8 > hcd->rsrc_len) {
		ret = -EIO;
		goto unlock;
	}
	if (readl(hcd->regs + caplen + 4) & 1) {	/* USBSTS.HCH */
		ret = -EIO;
		goto unlock;
	}
	src.mfindex = hcd->regs + rtsoff;
	r->unit_ns = HS_UNIT_NS;
	ret = time_steps(&src, r->edges, (u64)(r->edges + 2) * HS_UNIT_NS, r->irq_off_max_ns, r);

unlock:
	device_unlock(&pdev->dev);
	pm_runtime_put(&pdev->dev);
put_dev:
	pci_dev_put(pdev);
	return ret;
}

struct root_hub_match {
	int busnum;
	struct usb_device *udev;
};

static int match_root_hub(struct usb_device *udev, void *data)
{
	struct root_hub_match *m = data;

	if (udev->parent || udev->bus->busnum != m->busnum)
		return 0;
	m->udev = usb_get_dev(udev);
	return 1;
}

static int sample_generic(struct sofclock_req *r)
{
	struct counter_src src = { .read = read_frame_number };
	struct root_hub_match m = { .busnum = r->busnum };
	struct usb_device *root_hub;
	int ret;

	if (r->busnum < 1 || r->busnum > INT_MAX)
		return -EINVAL;
	if (!usb_for_each_dev(&m, match_root_hub))
		return -ENODEV;
	root_hub = src.root_hub = m.udev;

	/* keeps the HCD from going away */
	usb_lock_device(root_hub);
	if (root_hub->state == USB_STATE_NOTATTACHED) {
		ret = -ENODEV;
		goto unlock;
	}
	r->unit_ns = 0;
	ret = time_steps(&src, r->edges, (u64)(r->edges + 2) * FRAME_NS, r->irq_off_max_ns, r);
unlock:
	usb_unlock_device(root_hub);
	usb_put_dev(root_hub);
	return ret;
}

static long sofclock_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sofclock_req r;
	int ret;

	if (cmd != SOFCLOCK_IOC_SAMPLE)
		return -ENOTTY;
	if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
		return -EFAULT;
	if (r.edges < 1 || r.edges > MAX_EDGES || r.irq_off_max_ns > MAX_IRQ_OFF_NS)
		return -EINVAL;

	switch (r.mode) {
	case SOFCLOCK_MODE_XHCI_PCI:
		ret = sample_xhci_pci(&r);
		break;
	case SOFCLOCK_MODE_GENERIC:
		ret = sample_generic(&r);
		break;
	default:
		return -EINVAL;
	}
	if (!ret && copy_to_user((void __user *)arg, &r, sizeof(r)))
		ret = -EFAULT;
	return ret;
}

static const struct file_operations sofclock_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sofclock_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static struct miscdevice sofclock_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sofclock",
	.fops = &sofclock_fops,
	.mode = 0600,
};

module_misc_device(sofclock_misc);

MODULE_DESCRIPTION("USB host controller frame counter timing for sofd");
MODULE_LICENSE("GPL");
