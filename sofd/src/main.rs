//! sofd: timestamps USB start-of-frame by sampling host controller frame counters.

mod backend;
mod chrony;
mod clock;
mod config;
mod daemon;
mod discovery;
mod gnss;
mod kmod;
mod mmio;
mod model;
mod mqtt;
mod power;
mod sampler;
mod uevent;

use anyhow::Result;
use clap::{Parser, Subcommand};
use config::{Attach, Config};
use daemon::{Daemon, Options};
use discovery::{DeviceQuery, MmioOverride};
use log::{info, warn};
use sampler::SamplerConfig;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

const DEFAULT_CONFIG: &str = "/etc/sofd/sofd.hujson";
/// Holds the last epoch (systemd StateDirectory=sofd).
const STATE_DIR: &str = "/var/lib/sofd";

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
    /// Register window for a controller discovery cannot place: ID=ADDR[:LEN] (hex)
    #[arg(long, global = true)]
    mmio: Vec<MmioOverride>,
    /// Add a simulated controller "sim0" for testing without hardware or root
    #[arg(long, global = true)]
    simulate: bool,
    /// Use the generic kernel fallback (usb_get_current_frame_number via
    /// sofclock) for this controller even if it has a register backend
    #[arg(long, global = true, value_name = "ID")]
    generic: Vec<String>,
    /// Let the sofclock module disable interrupts for up to this long while
    /// spinning (tighter brackets, but delays other devices' interrupts,
    /// e.g. SDRs). 0 = never
    #[arg(long, global = true, default_value_t = 0, value_parser = clap::value_parser!(u32).range(0..=500))]
    irq_off_max_us: u32,
}

#[derive(Subcommand)]
enum Cmd {
    /// Run as a service: use sofgnss boards, serve MQTT and chrony as configured
    Run {
        /// Configuration file (HuJSON); the default path may be absent
        #[arg(long)]
        config: Option<PathBuf>,
    },
    /// Print controllers and their devices as JSON
    List,
    /// Print the controller serving a device as JSON
    Lookup {
        /// vvvv:pppp
        #[arg(long)]
        device: Option<String>,
        #[arg(long)]
        serial: Option<String>,
        /// sysfs port path, e.g. 1-1.2
        #[arg(long)]
        port: Option<String>,
        #[arg(long)]
        busnum: Option<u32>,
        #[arg(long)]
        devnum: Option<u32>,
    },
    /// Sample one controller and print events to stdout (no MQTT)
    Probe {
        controller: String,
        #[arg(long, default_value_t = 10.0)]
        rate: f64,
        /// Stop after this many samples
        #[arg(long)]
        count: Option<u64>,
        /// Use a sofgnss board as GNSS reference: its tty, or "auto" for the
        /// board on this controller
        #[arg(long, value_name = "TTY|auto")]
        gnss: Option<String>,
        /// Board and sampler options from this configuration file
        #[arg(long)]
        config: Option<PathBuf>,
    },
}

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

/// So a kill still restores power settings and the MQTT state.
fn install_signal_handlers() {
    for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP, libc::SIGQUIT] {
        // SAFETY: the handler only stores to an atomic, which is async-signal-safe.
        unsafe { libc::signal(sig, on_signal as *const () as libc::sighandler_t) };
    }
}

fn load_config(path: Option<&Path>) -> Result<Config> {
    match path {
        Some(p) => Config::load(p),
        None if Path::new(DEFAULT_CONFIG).exists() => Config::load(Path::new(DEFAULT_CONFIG)),
        None => {
            info!("no {DEFAULT_CONFIG}, using defaults");
            Ok(Config::default())
        }
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .target(env_logger::Target::Stderr)
        .init();
    let cli = Cli::parse();
    let opts = |cfg: &Config| Options {
        overrides: cli.mmio.clone(),
        simulate: cli.simulate,
        force_generic: cli.generic.clone(),
        irq_off_max_ns: cli.irq_off_max_us * 1000,
        sampler: SamplerConfig { window: cfg.sampler.window, rt_prio: cfg.sampler.rt_prio, cpu: cfg.sampler.cpu },
        default_rate_hz: cfg.sampler.default_rate_hz,
        model_hz: cfg.sampler.model_hz,
        power: power::Settings {
            max_cpu_latency_us: cfg.sampler.max_cpu_latency_us,
            cpu_governor: cfg.sampler.cpu_governor.clone(),
            cpu: cfg.sampler.cpu,
        },
    };

    match &cli.cmd {
        Cmd::List => println!("{}", serde_json::to_string_pretty(&daemon::list(cli.simulate)?)?),
        Cmd::Lookup { device, serial, port, busnum, devnum } => {
            let q = DeviceQuery {
                device: device.clone(),
                serial: serial.clone(),
                port: port.clone(),
                busnum: *busnum,
                devnum: *devnum,
            };
            println!("{}", serde_json::to_string_pretty(&daemon::lookup(&q, cli.simulate)?)?);
        }
        Cmd::Probe { controller, rate, count, gnss, config } => {
            let mut cfg = match config {
                Some(p) => Config::load(p)?,
                None => Config::default(),
            };
            cfg.gnss.attach = match gnss {
                Some(spec) => Attach::Serials(vec![daemon::board_serial(spec, controller, cli.simulate)?]),
                None => Attach::None,
            };
            lock_memory(cfg.sampler.rt_prio);
            install_signal_handlers();
            let (tx, rx) = std::sync::mpsc::channel();
            let mut d = Daemon::new(opts(&cfg), cfg.gnss, tx, None, true);
            d.want(controller, "probe", *rate);
            d.sample_limit = *count;
            d.run(&rx, &STOP);
        }
        Cmd::Run { config } => {
            let cfg = load_config(config.as_deref())?;
            match std::fs::create_dir_all(STATE_DIR) {
                Ok(()) => sampler::persist_epochs(Path::new(STATE_DIR).join("epoch")),
                Err(e) => warn!("{STATE_DIR}: {e}; epochs may repeat after a restart with the clock set back"),
            }
            lock_memory(cfg.sampler.rt_prio);
            install_signal_handlers();
            let (tx, rx) = std::sync::mpsc::channel();
            let mqtt = cfg.mqtt.as_ref().map(|m| mqtt::Mqtt::connect(m, tx.clone())).transpose()?;
            if mqtt.is_none() {
                info!("no mqtt section: not connecting to a broker");
            }
            Daemon::new(opts(&cfg), cfg.gnss, tx, mqtt, false).run(&rx, &STOP);
            info!("stopped");
        }
    }
    Ok(())
}

/// No page faults in the timing loops when real-time.
fn lock_memory(rt_prio: Option<i32>) {
    // SAFETY: mlockall has no memory-safety preconditions.
    if rt_prio.is_some() && unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } != 0 {
        warn!("mlockall: {}", std::io::Error::last_os_error());
    }
}
