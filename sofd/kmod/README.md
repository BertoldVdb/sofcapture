# sofclock kernel module

This module lets [sofd](..) time the steps of a USB host controller's frame
counter from inside the kernel. It creates `/dev/sofclock`, which only root
can open, with a single ioctl. You need it when:

- the kernel will not let user space map the controller's registers
  (`CONFIG_IO_STRICT_DEVMEM`, used by most Debian/Ubuntu desktop kernels, or
  lockdown) and you don't want to boot with `iomem=relaxed`, or
- the controller's driver is not one sofd has a register backend for (sofd
  supports xHCI and DWC2).

The Raspberry Pi and the Jetson don't need it. sofd uses the module
automatically when it is loaded. `sofd list` reports `"supported": true` for
a controller once sofd can sample it.

## Modes

- In xHCI (PCI) mode the module times `MFINDEX` steps (125 µs microframes)
  through the register mapping of the bound `xhci_hcd` driver. The device lock keeps the driver
  from unbinding during a read.
- Generic mode polls `usb_get_current_frame_number()` on a controller's root
  hub, so it works with any driver. Most drivers count 1 ms frames, so a
  sample spins for up to about 1 ms. sofd measures the tick length and handles
  the driver-specific wrap point. Generic mode is used only for drivers sofd
  doesn't know, or when forced with `sofd --generic ID`.

Interrupts stay enabled while spinning, because busy devices such as SDRs need
their USB interrupts serviced quickly. A step whose bracket (the time between
the reads on either side of it) is longer than 4 fastest reads + 1 µs is
discarded and the spin continues. This rejects interrupts, NMIs, SMIs and bus
stalls alike. `sofd --irq-off-max-us N` (N ≤ 500) lets the module disable
interrupts in slices of at most N µs. On the test PC that made no measurable
difference.

## Build and load

You need the headers of the running kernel (`linux-headers-$(uname -r)` on
Debian/Ubuntu):

```sh
make                              # or: make KDIR=/path/to/kernel/build
sudo insmod sofclock.ko
sudo sofd probe 0000:00:14.0      # logs "using the sofclock kernel module"
```

With Secure Boot enabled, the kernel only loads signed modules. Sign the
module with a key enrolled as a MOK, using the kernel's `scripts/sign-file`:

```sh
sudo /lib/modules/$(uname -r)/build/scripts/sign-file sha256 MOK.priv MOK.der sofclock.ko
```

To load it at boot:

```sh
sudo make -C /lib/modules/$(uname -r)/build M=$PWD modules_install
sudo depmod -a
echo sofclock | sudo tee /etc/modules-load.d/sofclock.conf
```

The module is built for one kernel. Rebuild and reinstall it after every
kernel update.

## Interface

`sofclock.h` holds the user API. sofd's `src/kmod.rs` mirrors it.
`SOFCLOCK_IOC_SAMPLE` takes a `struct sofclock_req` that names a controller
(a PCI address for xHCI, a USB bus number for generic mode) and the number of
steps to time (1–4). It returns the counter value after the tightest step,
the `CLOCK_MONOTONIC_RAW` time of that step, and half its bracket width.

| Error | Meaning |
|---|---|
| `ENODEV` | no such controller, or no driver bound |
| `EOPNOTSUPP` | xHCI mode on a controller that is not xHCI |
| `EIO` | counter not running |
| `EAGAIN` | no clean step within the time budget, try again |
| `EINVAL` | bad mode, `edges` or `irq_off_max_ns` |

Measurements and how sofd picks a backend for each controller are in
[sofd's README](../README.md#sofclock-kernel-module-pcs-unknown-controllers).
