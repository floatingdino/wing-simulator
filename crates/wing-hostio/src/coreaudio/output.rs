//! [`HostOutput`]: runs a [`BridgeConsumer`] in a HAL IO proc on a named device, and keeps it
//! running across device disconnects, sample-rate and buffer-size changes.
//!
//! Threads:
//! * HAL IO thread: [`io_proc`] -> [`BridgeConsumer::render`] -> device buffers. This thread is
//!   created by the HAL, runs with a time-constraint policy, and is already a member of the
//!   device's IO `os_workgroup`, so nothing needs joining there. [`HostOutput::io_workgroup`]
//!   hands the workgroup out for any helper thread that works on this device's deadline.
//! * Supervisor thread (non-RT): opens the device, and on HAL notifications (device list,
//!   `IsAlive`, nominal rate, buffer size) or a 500 ms poll, tears down and re-opens as needed.
//!
//! The consumer is owned by the supervisor and lent to the IO proc only between
//! `AudioDeviceStart` and `AudioDeviceStop` (which blocks until the IO proc has returned).

use std::ffi::c_void;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use super::device::{self, addr, check, CaError, DeviceInfo, DeviceWorkgroup};
use super::sys::*;
use crate::bridge::{BridgeConsumer, BridgeStats};

/// What to open.
#[derive(Debug, Clone, PartialEq)]
pub struct HostOutputConfig {
    /// Device name or UID (see [`device::find_output_device`]).
    pub device: String,
    /// 1-based device output channel for each bridge channel.
    pub device_channels: Vec<u16>,
    /// Optional IO buffer size request (frames).
    pub buffer_frames: Option<u32>,
}

/// Events for the supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Event {
    Changed,
    Shutdown,
}

/// Per-open IO proc context (lives in a Box while the IO proc may run).
struct IoCtx {
    consumer: BridgeConsumer,
    /// For each bridge channel: (buffer index, channel offset in buffer, buffer channel count).
    map: Vec<(usize, usize, usize)>,
    expected_buffers: usize,
    timebase: (u32, u32),
}

/// Currently open device.
struct Open {
    info: DeviceInfo,
    proc_id: AudioDeviceIOProcID,
    ctx: *mut IoCtx,
    listener: *mut Sender<Event>,
}

/// Device properties we watch while open.
const WATCHED: [u32; 3] = [
    kAudioDevicePropertyDeviceIsAlive,
    kAudioDevicePropertyNominalSampleRate,
    kAudioDevicePropertyBufferFrameSize,
];

/// Status shared with the [`HostOutput`] handle.
#[derive(Debug, Clone, Default)]
struct Shared {
    device: Option<DeviceInfo>,
    last_error: Option<String>,
}

/// A running host output. Dropping it stops the device IO and the supervisor.
pub struct HostOutput {
    tx: Sender<Event>,
    thread: Option<JoinHandle<()>>,
    stats: Arc<BridgeStats>,
    shared: Arc<Mutex<Shared>>,
}

impl HostOutput {
    /// Start the output. Fails only on invalid config; a missing device is retried in the
    /// background (see [`HostOutput::device`] / [`HostOutput::last_error`]). `wait` > 0 waits
    /// up to that long for the first open and returns its error, if any.
    pub fn start(
        cfg: HostOutputConfig,
        consumer: BridgeConsumer,
        wait: Duration,
    ) -> Result<Self, CaError> {
        if cfg.device_channels.len() != consumer.channels() {
            return Err(CaError {
                what: format!(
                    "{} device channels for a {}-channel bridge",
                    cfg.device_channels.len(),
                    consumer.channels()
                ),
                status: 0,
            });
        }
        if cfg.device_channels.contains(&0) {
            return Err(CaError {
                what: "device channels are 1-based".into(),
                status: 0,
            });
        }
        device::use_hal_notification_thread();
        let stats = consumer.stats().clone();
        let shared = Arc::new(Mutex::new(Shared::default()));
        let (tx, rx) = mpsc::channel();
        let (first_tx, first_rx) = mpsc::channel();
        let sup = Supervisor {
            cfg,
            consumer: Some(consumer),
            open: None,
            tx: tx.clone(),
            shared: shared.clone(),
            first: Some(first_tx),
        };
        let thread = std::thread::Builder::new()
            .name("hostio-supervisor".into())
            .spawn(move || sup.run(rx))
            .map_err(|e| CaError {
                what: format!("spawn supervisor: {e}"),
                status: 0,
            })?;
        let out = Self {
            tx,
            thread: Some(thread),
            stats,
            shared,
        };
        if !wait.is_zero() {
            if let Ok(Err(e)) = first_rx.recv_timeout(wait) {
                return Err(e);
            }
        }
        Ok(out)
    }

    /// Bridge statistics.
    pub fn stats(&self) -> &Arc<BridgeStats> {
        &self.stats
    }

    /// The device currently open (None while disconnected).
    pub fn device(&self) -> Option<DeviceInfo> {
        self.shared.lock().ok()?.device.clone()
    }

    /// Last open error (cleared on success).
    pub fn last_error(&self) -> Option<String> {
        self.shared.lock().ok()?.last_error.clone()
    }

    /// The open device's IO workgroup (for helper threads working on its deadline).
    pub fn io_workgroup(&self) -> Option<DeviceWorkgroup> {
        DeviceWorkgroup::of_device(self.device()?.id)
    }
}

impl Drop for HostOutput {
    fn drop(&mut self) {
        let _ = self.tx.send(Event::Shutdown);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct Supervisor {
    cfg: HostOutputConfig,
    consumer: Option<BridgeConsumer>,
    open: Option<Open>,
    tx: Sender<Event>,
    shared: Arc<Mutex<Shared>>,
    first: Option<Sender<Result<(), CaError>>>,
}

// SAFETY: raw pointers in `Open` are owned by the supervisor and only shared with the HAL.
unsafe impl Send for Supervisor {}

unsafe extern "C" fn listener(
    _obj: AudioObjectID,
    _n: u32,
    _addrs: *const AudioObjectPropertyAddress,
    client: *mut c_void,
) -> OSStatus {
    // SAFETY: `client` is a Box<Sender<Event>> kept alive until the listener is removed.
    let tx = unsafe { &*(client as *const Sender<Event>) };
    let _ = tx.send(Event::Changed);
    0
}

impl Supervisor {
    fn run(mut self, rx: Receiver<Event>) {
        // System-wide device list listener.
        let sys_listener = Box::into_raw(Box::new(self.tx.clone()));
        let dev_addr = addr(
            kAudioHardwarePropertyDevices,
            kAudioObjectPropertyScopeGlobal,
        );
        // SAFETY: listener + client pointer stay valid until removed below.
        unsafe {
            AudioObjectAddPropertyListener(
                kAudioObjectSystemObject,
                &dev_addr,
                listener,
                sys_listener as *mut c_void,
            );
        }
        self.reconcile();
        loop {
            match rx.recv_timeout(Duration::from_millis(500)) {
                Ok(Event::Shutdown) | Err(RecvTimeoutError::Disconnected) => break,
                Ok(Event::Changed) => {
                    // Coalesce bursts of notifications.
                    std::thread::sleep(Duration::from_millis(50));
                    while let Ok(e) = rx.try_recv() {
                        if e == Event::Shutdown {
                            self.close();
                            self.remove_sys_listener(sys_listener);
                            return;
                        }
                    }
                    self.reconcile();
                }
                Err(RecvTimeoutError::Timeout) => self.reconcile(),
            }
        }
        self.close();
        self.remove_sys_listener(sys_listener);
    }

    fn remove_sys_listener(&self, l: *mut Sender<Event>) {
        let dev_addr = addr(
            kAudioHardwarePropertyDevices,
            kAudioObjectPropertyScopeGlobal,
        );
        // SAFETY: removing what we added; afterwards the HAL no longer calls it.
        unsafe {
            AudioObjectRemovePropertyListener(
                kAudioObjectSystemObject,
                &dev_addr,
                listener,
                l as *mut c_void,
            );
            drop(Box::from_raw(l));
        }
    }

    /// Make reality match the config: close a dead/changed device, open when available.
    fn reconcile(&mut self) {
        if let Some(o) = &self.open {
            let id = o.info.id;
            let changed = !device::is_alive(id)
                || !device::nominal_rate(id).is_ok_and(|r| (r - o.info.sample_rate).abs() <= 0.5)
                || device::buffer_frames(id) != Ok(o.info.buffer_frames)
                || !device::device_ids().is_ok_and(|ids| ids.contains(&id));
            if !changed {
                return;
            }
            self.close();
        }
        let res = self.try_open();
        if let Ok(mut s) = self.shared.lock() {
            match &res {
                Ok(()) => s.last_error = None,
                Err(e) => s.last_error = Some(e.to_string()),
            }
            s.device = self.open.as_ref().map(|o| o.info.clone());
        }
        if let Some(f) = self.first.take() {
            let _ = f.send(res);
        }
    }

    fn try_open(&mut self) -> Result<(), CaError> {
        let not_found = || CaError {
            what: format!("output device '{}' not found", self.cfg.device),
            status: 0,
        };
        let mut info = device::find_output_device(&self.cfg.device)?.ok_or_else(not_found)?;
        let max_ch = *self.cfg.device_channels.iter().max().unwrap_or(&1);
        if u32::from(max_ch) > info.output_channels {
            return Err(CaError {
                what: format!(
                    "'{}' has {} output channels, route needs channel {max_ch}",
                    info.name, info.output_channels
                ),
                status: 0,
            });
        }
        if !device::outputs_are_f32(info.id)? {
            return Err(CaError {
                what: format!("'{}' output format is not 32-bit float", info.name),
                status: 0,
            });
        }
        if let Some(b) = self.cfg.buffer_frames {
            if b != info.buffer_frames {
                device::set_buffer_frames(info.id, b)?;
                info.buffer_frames = device::buffer_frames(info.id)?;
            }
        }
        // Map device channels onto (buffer, offset) via the output stream layout.
        let layout = device::stream_layout(info.id, kAudioObjectPropertyScopeOutput)?;
        let mut map = Vec::with_capacity(self.cfg.device_channels.len());
        for &dc in &self.cfg.device_channels {
            let mut idx = usize::from(dc) - 1;
            let mut found = None;
            for (b, &n) in layout.iter().enumerate() {
                if idx < n as usize {
                    found = Some((b, idx, n as usize));
                    break;
                }
                idx -= n as usize;
            }
            map.push(found.ok_or_else(|| CaError {
                what: format!("channel {dc} not in output stream layout"),
                status: 0,
            })?);
        }

        let mut consumer = self.consumer.take().expect("consumer present while closed");
        consumer.configure(info.sample_rate, info.buffer_frames.max(1) as usize);
        // Output side of the end-to-end latency: device latency + safety offset + one buffer.
        let out_frames = device::output_latency_frames(info.id) + info.buffer_frames;
        consumer.stats().set_output_latency(Duration::from_secs_f64(
            out_frames as f64 / info.sample_rate.max(1.0),
        ));
        consumer.start();
        let ctx = Box::into_raw(Box::new(IoCtx {
            consumer,
            map,
            expected_buffers: layout.len(),
            timebase: timebase(),
        }));
        let mut proc_id: AudioDeviceIOProcID = None;
        // SAFETY: `ctx` stays valid until the IO proc is destroyed in `close`.
        let st = unsafe {
            AudioDeviceCreateIOProcID(info.id, io_proc, ctx as *mut c_void, &mut proc_id)
        };
        if let Err(e) = check(st, || format!("create IO proc on '{}'", info.name)) {
            // SAFETY: the HAL never saw `ctx`.
            let mut ctx = unsafe { Box::from_raw(ctx) };
            ctx.consumer.stop();
            self.consumer = Some(ctx.consumer);
            return Err(e);
        }
        let l = Box::into_raw(Box::new(self.tx.clone()));
        for sel in WATCHED {
            let a = addr(sel, kAudioObjectPropertyScopeGlobal);
            // SAFETY: removed in `close` before `l` is freed.
            unsafe { AudioObjectAddPropertyListener(info.id, &a, listener, l as *mut c_void) };
        }
        self.open = Some(Open {
            info: info.clone(),
            proc_id,
            ctx,
            listener: l,
        });
        // SAFETY: valid device and proc id.
        let st = unsafe { AudioDeviceStart(info.id, proc_id) };
        if let Err(e) = check(st, || format!("start '{}'", info.name)) {
            self.close();
            return Err(e);
        }
        // SAFETY: ctx valid; stats are atomics.
        unsafe { (*ctx).consumer.stats().note_reconnect() };
        Ok(())
    }

    fn close(&mut self) {
        let Some(o) = self.open.take() else { return };
        // SAFETY: AudioDeviceStop from a non-IO thread returns after the IO proc has finished;
        // after DestroyIOProcID the HAL holds no reference to `ctx`.
        unsafe {
            AudioDeviceStop(o.info.id, o.proc_id);
            AudioDeviceDestroyIOProcID(o.info.id, o.proc_id);
            for sel in WATCHED {
                let a = addr(sel, kAudioObjectPropertyScopeGlobal);
                AudioObjectRemovePropertyListener(
                    o.info.id,
                    &a,
                    listener,
                    o.listener as *mut c_void,
                );
            }
            drop(Box::from_raw(o.listener));
            let mut ctx = Box::from_raw(o.ctx);
            ctx.consumer.stop();
            self.consumer = Some(ctx.consumer);
        }
        if let Ok(mut s) = self.shared.lock() {
            s.device = None;
        }
    }
}

fn timebase() -> (u32, u32) {
    // Derive from the ns conversion helper: ns(1e9 ticks) / 1e9 as a ratio.
    let ns = device::host_ticks_to_ns(1_000_000_000);
    // Reduce ns/1e9 to a small fraction (exact for the 1/1 and 125/3 timebases).
    let g = gcd(ns, 1_000_000_000);
    ((ns / g) as u32, (1_000_000_000 / g) as u32)
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// The HAL IO proc. RT thread: no allocation, locks or syscalls.
unsafe extern "C" fn io_proc(
    _dev: AudioObjectID,
    now: *const AudioTimeStamp,
    _in: *const AudioBufferList,
    _in_time: *const AudioTimeStamp,
    out: *mut AudioBufferList,
    _out_time: *const AudioTimeStamp,
    client: *mut c_void,
) -> OSStatus {
    if out.is_null() || client.is_null() {
        return 0;
    }
    // SAFETY: `client` is the IoCtx registered with this proc; only this thread touches it
    // while the device runs. `out` is a valid AudioBufferList with mNumberBuffers entries.
    unsafe {
        let ctx = &mut *(client as *mut IoCtx);
        let nbuf = (*out).mNumberBuffers as usize;
        let bufs = std::slice::from_raw_parts_mut((*out).mBuffers.as_mut_ptr(), nbuf);
        let mut frames = usize::MAX;
        for b in bufs.iter_mut() {
            if !b.mData.is_null() {
                std::ptr::write_bytes(b.mData as *mut u8, 0, b.mDataByteSize as usize);
                let ch = b.mNumberChannels.max(1) as usize;
                frames = frames.min(b.mDataByteSize as usize / (4 * ch));
            }
        }
        if frames == usize::MAX || frames == 0 || nbuf < ctx.expected_buffers {
            return 0;
        }
        let now_ns = if !now.is_null() && (*now).mFlags & kAudioTimeStampHostTimeValid != 0 {
            let (n, d) = ctx.timebase;
            ((*now).mHostTime as u128 * n as u128 / d as u128) as u64
        } else {
            0
        };
        let map = &ctx.map;
        ctx.consumer.render(frames, now_ns, |i, samples| {
            for (s, &(b, off, stride)) in samples.iter().zip(map.iter()) {
                let buf = &bufs[b];
                if !buf.mData.is_null() {
                    *(buf.mData as *mut f32).add(i * stride + off) = *s;
                }
            }
        });
    }
    0
}
