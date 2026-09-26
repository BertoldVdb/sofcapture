//! sofd: timestamps USB start-of-frame by sampling host controller frame counters.

mod api;
mod backend;
mod clock;
mod discovery;
mod kmod;
mod mmio;
mod model;
mod sampler;

use anyhow::{Context, Result};
use api::{Daemon, Msg, Options};
use clap::{Args, Parser, Subcommand};
use discovery::{DeviceQuery, MmioOverride};
use log::{info, warn};
use sampler::SamplerConfig;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::Duration;

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

#[derive(Args, Clone)]
struct SamplerArgs {
    /// Samples kept in the rate/offset fit
    #[arg(long, default_value_t = 64)]
    window: usize,
    /// Run sampler threads SCHED_FIFO at this priority
    #[arg(long)]
    rt_prio: Option<i32>,
    /// Pin sampler threads to this CPU
    #[arg(long)]
    cpu: Option<usize>,
}

impl SamplerArgs {
    fn config(&self) -> SamplerConfig {
        SamplerConfig { window: self.window, rt_prio: self.rt_prio, cpu: self.cpu }
    }
}

#[derive(Subcommand)]
enum Cmd {
    /// Connect to the peer's Unix socket and serve its requests; exits when the
    /// connection fails, closes, or the peer stops pinging
    Run {
        /// Peer socket path
        #[arg(long)]
        connect: PathBuf,
        /// Close the connection (and exit) if no ping arrives for this many seconds
        #[arg(long, default_value_t = 30.0)]
        ping_timeout: f64,
        /// Rate used when a subscribe request gives none
        #[arg(long, default_value_t = 10.0)]
        default_rate: f64,
        #[command(flatten)]
        sampler: SamplerArgs,
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
    /// Sample one controller and print events to stdout
    Probe {
        controller: String,
        #[arg(long, default_value_t = 10.0)]
        rate: f64,
        /// Stop after this many samples
        #[arg(long)]
        count: Option<u64>,
        #[command(flatten)]
        sampler: SamplerArgs,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .target(env_logger::Target::Stderr)
        .init();
    let cli = Cli::parse();
    let opts = |default_rate_hz: f64, sampler: SamplerConfig| Options {
        overrides: cli.mmio.clone(),
        simulate: cli.simulate,
        force_generic: cli.generic.clone(),
        irq_off_max_ns: cli.irq_off_max_us * 1000,
        default_rate_hz,
        sampler,
    };

    match &cli.cmd {
        Cmd::List => {
            let d = Daemon::new(opts(10.0, SamplerArgs { window: 64, rt_prio: None, cpu: None }.config()));
            println!("{}", serde_json::to_string_pretty(&d.list()?)?);
        }
        Cmd::Lookup { device, serial, port, busnum, devnum } => {
            let d = Daemon::new(opts(10.0, SamplerArgs { window: 64, rt_prio: None, cpu: None }.config()));
            let q = DeviceQuery {
                device: device.clone(),
                serial: serial.clone(),
                port: port.clone(),
                busnum: *busnum,
                devnum: *devnum,
            };
            println!("{}", serde_json::to_string_pretty(&d.lookup(&q)?)?);
        }
        Cmd::Probe { controller, rate, count, sampler } => {
            lock_memory(sampler.rt_prio);
            let mut d = Daemon::new(opts(*rate, sampler.config()));
            let (tx, rx) = std::sync::mpsc::channel();
            d.subscribe(controller, Some(*rate), &tx)?;
            let mut samples = 0;
            for msg in rx {
                if let Msg::Out(line) = msg {
                    println!("{line}");
                    if line.contains(r#""event":"sample""#) {
                        samples += 1;
                        if count.is_some_and(|c| samples >= c) {
                            break;
                        }
                    }
                }
            }
        }
        Cmd::Run { connect, ping_timeout, default_rate, sampler } => {
            if !(ping_timeout.is_finite() && *ping_timeout > 0.0) {
                anyhow::bail!("--ping-timeout must be a positive number of seconds");
            }
            lock_memory(sampler.rt_prio);
            let mut d = Daemon::new(opts(*default_rate, sampler.config()));
            let stream = UnixStream::connect(connect)
                .with_context(|| format!("connecting to peer at {}", connect.display()))?;
            info!("connected to {}", connect.display());
            api::run_session(stream, &mut d, Duration::from_secs_f64(*ping_timeout))?;
            info!("peer disconnected");
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
