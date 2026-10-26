//! Mapping the POSIX shm segment: attach/create semantics matching the
//! driver's `ShmLink::Open` (non-real-time). The driver is macOS-only; on
//! non-Unix targets named segments are process-local heap allocations, so the
//! daemon finds no driver (runs control-only) while in-process tests still work.

#[cfg(unix)]
use std::ffi::CString;
use std::fmt;
use std::io;
use std::ptr::NonNull;
#[cfg(unix)]
use std::sync::atomic::Ordering::Acquire;
use std::sync::atomic::Ordering::Relaxed;
use std::time::Duration;

use crate::clock;
#[cfg(unix)]
use crate::layout::HEADER_BYTES;
use crate::layout::{Header, LayoutError, TOTAL_BYTES};

/// How [`Segment::open`] treats a missing or incompatible segment.
#[derive(Debug, Clone)]
pub struct OpenOptions {
    /// Create the segment if it does not exist (daemon started before the
    /// driver, or tests without a driver). The driver attaches to it later.
    pub create: bool,
    /// If an existing segment is incompatible (other version/size), unlink it
    /// and create a fresh one, like the driver does. A driver still mapping
    /// the old segment keeps using it until its next IO start (when it
    /// re-attaches by name, T021), so the daemon only does this when asked.
    pub replace_incompatible: bool,
    /// Nominal rate stored when creating.
    pub sample_rate: u32,
    /// How long to wait for a concurrent creator to publish its header.
    pub creator_grace: Duration,
}

impl Default for OpenOptions {
    fn default() -> Self {
        Self {
            create: false,
            replace_incompatible: false,
            sample_rate: 48_000,
            creator_grace: Duration::from_millis(100),
        }
    }
}

/// Errors from [`Segment::open`].
#[derive(Debug)]
pub enum Error {
    /// Name has an interior NUL or is longer than 31 bytes.
    BadName,
    /// The segment does not exist and `create` was false.
    NotFound,
    /// The segment exists but its header does not match this build.
    Incompatible(LayoutError),
    /// A system call failed.
    Io(&'static str, io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::BadName => write!(f, "invalid shm name (max 31 bytes, no NUL)"),
            Error::NotFound => write!(f, "shm segment not found (driver not loaded?)"),
            Error::Incompatible(e) => write!(f, "incompatible shm segment: {e:?}"),
            Error::Io(op, e) => write!(f, "{op}: {e}"),
        }
    }
}

impl std::error::Error for Error {}

/// A mapped wing_shm segment. Unmapped on drop (never unlinked).
pub struct Segment {
    base: NonNull<u8>,
    #[cfg_attr(not(unix), allow(dead_code))]
    len: usize,
    name: String,
    created: bool,
}

// SAFETY: all shared state is accessed through atomics (header) or raw
// pointer copies under the single-producer ring protocol.
unsafe impl Send for Segment {}
unsafe impl Sync for Segment {}

#[cfg(unix)]
fn errno() -> io::Error {
    io::Error::last_os_error()
}

#[cfg(unix)]
fn cname(name: &str) -> Result<CString, Error> {
    if name.len() > 31 {
        return Err(Error::BadName);
    }
    CString::new(name).map_err(|_| Error::BadName)
}

#[cfg(unix)]
/// Map `len` bytes of `fd` shared. Closes nothing.
fn map(fd: libc::c_int, len: usize, prot: libc::c_int) -> Result<NonNull<u8>, Error> {
    // SAFETY: plain mmap of a file descriptor we own.
    let p = unsafe { libc::mmap(std::ptr::null_mut(), len, prot, libc::MAP_SHARED, fd, 0) };
    if p == libc::MAP_FAILED {
        return Err(Error::Io("mmap", errno()));
    }
    Ok(NonNull::new(p.cast()).expect("mmap returned null"))
}

#[cfg(unix)]
fn fd_size(fd: libc::c_int) -> Result<u64, Error> {
    // SAFETY: zeroed stat is a valid out buffer.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    if unsafe { libc::fstat(fd, &mut st) } != 0 {
        return Err(Error::Io("fstat", errno()));
    }
    Ok(st.st_size as u64)
}

impl Segment {
    #[cfg(unix)]
    /// Attach to `name` if it holds a compatible segment; otherwise create or
    /// replace it as `opts` allows. Mirrors the driver's `ShmLink::Open`:
    /// mode 0666 (pshm ignores umask and cannot be fchmod'ed), `ftruncate`
    /// exactly once, header `magic` published last.
    pub fn open(name: &str, opts: &OpenOptions) -> Result<Segment, Error> {
        clock::init();
        let cn = cname(name)?;
        // A few rounds cover create/create races (EEXIST -> attach).
        for _ in 0..4 {
            // SAFETY: valid C string.
            let fd = unsafe { libc::shm_open(cn.as_ptr(), libc::O_RDWR, 0 as libc::c_uint) };
            if fd >= 0 {
                let res = Self::attach_fd(fd, name, opts);
                // SAFETY: fd is ours.
                unsafe { libc::close(fd) };
                match res {
                    Ok(seg) => return Ok(seg),
                    Err(Error::Incompatible(e)) if opts.replace_incompatible => {
                        let _ = e;
                        // SAFETY: valid C string.
                        unsafe { libc::shm_unlink(cn.as_ptr()) };
                    }
                    Err(e) => return Err(e),
                }
            } else {
                let err = errno();
                if err.raw_os_error() != Some(libc::ENOENT) {
                    return Err(Error::Io("shm_open", err));
                }
                if !opts.create {
                    return Err(Error::NotFound);
                }
            }
            match Self::create(&cn, name, opts) {
                Ok(seg) => return Ok(seg),
                Err(Error::Io(_, e)) if e.raw_os_error() == Some(libc::EEXIST) => continue,
                Err(e) => return Err(e),
            }
        }
        Err(Error::Io(
            "shm_open",
            io::Error::other("segment kept changing while opening"),
        ))
    }

    #[cfg(unix)]
    fn attach_fd(fd: libc::c_int, name: &str, opts: &OpenOptions) -> Result<Segment, Error> {
        let deadline =
            clock::now_ticks() + clock::ns_to_ticks(opts.creator_grace.as_nanos() as u64);
        // A creator may be between shm_open and ftruncate (size 0) or between
        // ftruncate and publishing magic: wait for it, briefly.
        let mut size = fd_size(fd)?;
        while size == 0 && clock::now_ticks() < deadline {
            std::thread::sleep(Duration::from_millis(1));
            size = fd_size(fd)?;
        }
        if size < TOTAL_BYTES as u64 {
            return Err(Error::Incompatible(LayoutError::TooSmall));
        }
        let base = map(fd, TOTAL_BYTES, libc::PROT_READ | libc::PROT_WRITE)?;
        let seg = Segment {
            base,
            len: TOTAL_BYTES,
            name: name.to_owned(),
            created: false,
        };
        loop {
            match seg.header().validate(size) {
                Ok(()) => return Ok(seg),
                Err(LayoutError::BadMagic(0)) if clock::now_ticks() < deadline => {
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => return Err(Error::Incompatible(e)),
            }
        }
    }

    #[cfg(unix)]
    fn create(cn: &CString, name: &str, opts: &OpenOptions) -> Result<Segment, Error> {
        // SAFETY: valid C string.
        let fd = unsafe {
            libc::shm_open(
                cn.as_ptr(),
                libc::O_RDWR | libc::O_CREAT | libc::O_EXCL,
                0o666 as libc::c_uint,
            )
        };
        if fd < 0 {
            return Err(Error::Io("shm_open(create)", errno()));
        }
        let fail = |op: &'static str| {
            let e = errno();
            // SAFETY: fd and name are ours.
            unsafe {
                libc::close(fd);
                libc::shm_unlink(cn.as_ptr());
            }
            Error::Io(op, e)
        };
        // SAFETY: fd is a fresh pshm object.
        if unsafe { libc::ftruncate(fd, TOTAL_BYTES as libc::off_t) } != 0 {
            return Err(fail("ftruncate"));
        }
        let base = match map(fd, TOTAL_BYTES, libc::PROT_READ | libc::PROT_WRITE) {
            Ok(b) => b,
            Err(_) => return Err(fail("mmap")),
        };
        // SAFETY: fd is ours.
        unsafe { libc::close(fd) };
        // SAFETY: fresh mapping of TOTAL_BYTES; new pshm objects are zero-filled
        // but clear explicitly like the driver does.
        unsafe {
            std::ptr::write_bytes(base.as_ptr(), 0, TOTAL_BYTES);
            Header::init(
                base.as_ptr().cast(),
                opts.sample_rate,
                clock::now_ticks(),
                std::process::id() as u64,
            );
        }
        Ok(Segment {
            base,
            len: TOTAL_BYTES,
            name: name.to_owned(),
            created: true,
        })
    }

    #[cfg(unix)]
    /// A private (anonymous, not shared between processes) segment with the
    /// same layout, for in-process tests.
    pub fn anonymous(sample_rate: u32) -> io::Result<Segment> {
        clock::init();
        // SAFETY: anonymous private mapping.
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                TOTAL_BYTES,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANON,
                -1,
                0,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(errno());
        }
        let base = NonNull::new(p.cast::<u8>()).expect("mmap returned null");
        // SAFETY: fresh zeroed mapping.
        unsafe {
            Header::init(
                base.as_ptr().cast(),
                sample_rate,
                clock::now_ticks(),
                std::process::id() as u64,
            )
        };
        Ok(Segment {
            base,
            len: TOTAL_BYTES,
            name: String::new(),
            created: true,
        })
    }

    #[cfg(unix)]
    /// Remove `name` from the shm namespace (existing mappings stay valid).
    pub fn unlink(name: &str) -> io::Result<()> {
        let cn = cname(name).map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))?;
        // SAFETY: valid C string.
        if unsafe { libc::shm_unlink(cn.as_ptr()) } != 0 {
            return Err(errno());
        }
        Ok(())
    }

    /// Without POSIX shm, named segments live in a process-local registry: the driver is
    /// macOS-only, so nothing outside this process could attach anyway, and in-process users
    /// (tests: `DriverSim` + `DaemonLink`) behave as on Unix. Registered memory is never freed
    /// (mappings stay valid after [`Segment::unlink`], as with shm).
    #[cfg(not(unix))]
    pub fn open(name: &str, opts: &OpenOptions) -> Result<Segment, Error> {
        clock::init();
        if name.len() > 31 || name.contains('\0') {
            return Err(Error::BadName);
        }
        let mut reg = registry();
        if let Some(i) = reg.iter().position(|(n, _)| n == name) {
            let seg = Segment {
                base: NonNull::new(reg[i].1 as *mut u8).expect("registered segment"),
                len: TOTAL_BYTES,
                name: name.to_owned(),
                created: false,
            };
            match seg.header().validate(TOTAL_BYTES as u64) {
                Ok(()) => return Ok(seg),
                Err(e) if !opts.replace_incompatible => return Err(Error::Incompatible(e)),
                Err(_) => {
                    reg.remove(i);
                }
            }
        }
        if !opts.create {
            return Err(Error::NotFound);
        }
        let mut seg = Self::anonymous(opts.sample_rate).map_err(|e| Error::Io("alloc", e))?;
        seg.name = name.to_owned();
        reg.push((name.to_owned(), seg.base.as_ptr() as usize));
        Ok(seg)
    }

    /// A private segment with the same layout, for in-process tests.
    #[cfg(not(unix))]
    pub fn anonymous(sample_rate: u32) -> io::Result<Segment> {
        clock::init();
        // SAFETY: non-zero size.
        let p = unsafe { std::alloc::alloc_zeroed(anon_layout()) };
        let base = NonNull::new(p).ok_or_else(|| io::Error::from(io::ErrorKind::OutOfMemory))?;
        // SAFETY: fresh zeroed allocation of TOTAL_BYTES.
        unsafe {
            Header::init(
                base.as_ptr().cast(),
                sample_rate,
                clock::now_ticks(),
                std::process::id() as u64,
            )
        };
        Ok(Segment {
            base,
            len: TOTAL_BYTES,
            name: String::new(),
            created: true,
        })
    }

    /// Remove `name` from the process-local registry (existing mappings stay valid).
    #[cfg(not(unix))]
    pub fn unlink(name: &str) -> io::Result<()> {
        let mut reg = registry();
        let i = reg
            .iter()
            .position(|(n, _)| n == name)
            .ok_or(io::ErrorKind::NotFound)?;
        reg.remove(i);
        Ok(())
    }

    /// False if `name` no longer refers to this segment.
    #[cfg(not(unix))]
    pub fn is_current(&self) -> bool {
        registry()
            .iter()
            .any(|(n, p)| *n == self.name && *p == self.base.as_ptr() as usize)
    }

    /// The validated header.
    #[inline]
    pub fn header(&self) -> &Header {
        // SAFETY: the mapping is at least HEADER_BYTES, aligned to a page and
        // Header is all atomics/immutable-after-publish fields.
        unsafe { &*self.base.as_ptr().cast::<Header>() }
    }

    /// Base address of the mapping.
    #[inline]
    pub fn base(&self) -> *mut u8 {
        self.base.as_ptr()
    }

    /// The shm name ("" for [`Segment::anonymous`]).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// True if this process created the segment.
    pub fn created(&self) -> bool {
        self.created
    }

    #[cfg(unix)]
    /// False if `name` no longer refers to this segment (unlinked, or
    /// recreated by the driver after a version change). Non-RT: opens and
    /// maps the name read-only.
    pub fn is_current(&self) -> bool {
        let Ok(cn) = cname(&self.name) else {
            return false;
        };
        // SAFETY: valid C string.
        let fd = unsafe { libc::shm_open(cn.as_ptr(), libc::O_RDONLY, 0 as libc::c_uint) };
        if fd < 0 {
            return false;
        }
        let same = match fd_size(fd) {
            Ok(sz) if sz >= HEADER_BYTES as u64 => match map(fd, HEADER_BYTES, libc::PROT_READ) {
                Ok(p) => {
                    // SAFETY: HEADER_BYTES mapped read-only.
                    let other = unsafe { &*p.as_ptr().cast::<Header>() };
                    let same = other.magic.load(Acquire) == self.header().magic.load(Acquire)
                        && other.segment_id == self.header().segment_id;
                    // SAFETY: unmapping what we mapped.
                    unsafe { libc::munmap(p.as_ptr().cast(), HEADER_BYTES) };
                    same
                }
                Err(_) => false,
            },
            _ => false,
        };
        // SAFETY: fd is ours.
        unsafe { libc::close(fd) };
        same
    }

    /// Snapshot of the header for diagnostics (non-RT).
    pub fn status(&self) -> Status {
        let h = self.header();
        let now = clock::now_ticks();
        let age = |hb: u64| (hb != 0).then(|| clock::ticks_to_ns(now.saturating_sub(hb)));
        Status {
            version: h.version,
            channels: h.channels,
            ring_frames: h.ring_frames,
            input_delay_frames: h.input_delay_frames,
            segment_id: h.segment_id,
            driver_pid: h.driver_pid.load(Relaxed),
            sample_rate: h.sample_rate.load(Relaxed),
            io_epoch: h.io_epoch.load(Relaxed),
            io_running: h.io_running.load(Relaxed) != 0,
            loopback_active: h.loopback_active.load(Relaxed) != 0,
            cycle_seq: h.cycle_seq.load(Relaxed),
            input_time: h.input_time.load(Relaxed),
            input_frames: h.input_frames.load(Relaxed),
            driver_heartbeat_age_ns: age(h.driver_heartbeat.load(Relaxed)),
            daemon_pid: h.daemon_pid.load(Relaxed),
            daemon_heartbeat_age_ns: age(h.daemon_heartbeat.load(Relaxed)),
            to_daemon: RingStatus::of(&h.to_daemon),
            from_daemon: RingStatus::of(&h.from_daemon),
        }
    }
}

impl Drop for Segment {
    fn drop(&mut self) {
        // SAFETY: unmapping our own mapping.
        #[cfg(unix)]
        unsafe {
            libc::munmap(self.base.as_ptr().cast(), self.len)
        };
        // SAFETY: allocated by `anonymous` with this layout; named segments are never freed.
        #[cfg(not(unix))]
        if self.name.is_empty() {
            unsafe { std::alloc::dealloc(self.base.as_ptr(), anon_layout()) };
        }
    }
}

/// Process-local named segments (name, base address); see [`Segment::open`].
#[cfg(not(unix))]
fn registry() -> std::sync::MutexGuard<'static, Vec<(String, usize)>> {
    static R: std::sync::Mutex<Vec<(String, usize)>> = std::sync::Mutex::new(Vec::new());
    R.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(not(unix))]
fn anon_layout() -> std::alloc::Layout {
    std::alloc::Layout::from_size_align(TOTAL_BYTES, 4096).expect("segment layout")
}

/// Snapshot of one ring control block.
#[derive(Debug, Clone, Copy)]
pub struct RingStatus {
    pub write_time: i64,
    pub read_time: i64,
    pub epoch: u32,
    pub overruns: u32,
    pub underruns: u32,
}

impl RingStatus {
    fn of(c: &crate::layout::RingCtl) -> Self {
        Self {
            write_time: c.write_time.load(Relaxed),
            read_time: c.read_time.load(Relaxed),
            epoch: c.epoch.load(Relaxed),
            overruns: c.overruns.load(Relaxed),
            underruns: c.underruns.load(Relaxed),
        }
    }
}

/// Snapshot of the header (see [`Segment::status`]).
#[derive(Debug, Clone)]
pub struct Status {
    pub version: u32,
    pub channels: u32,
    pub ring_frames: u32,
    pub input_delay_frames: u32,
    pub segment_id: u64,
    pub driver_pid: u64,
    pub sample_rate: u32,
    pub io_epoch: u32,
    pub io_running: bool,
    pub loopback_active: bool,
    pub cycle_seq: u32,
    pub input_time: i64,
    pub input_frames: u32,
    pub driver_heartbeat_age_ns: Option<u64>,
    pub daemon_pid: u64,
    pub daemon_heartbeat_age_ns: Option<u64>,
    pub to_daemon: RingStatus,
    pub from_daemon: RingStatus,
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let age = |a: Option<u64>| match a {
            None => "never".to_string(),
            Some(ns) => format!("{:.1} ms ago", ns as f64 / 1e6),
        };
        writeln!(
            f,
            "segment        v{}, {} ch, ring {} frames, input delay {} frames, id {:#x}",
            self.version, self.channels, self.ring_frames, self.input_delay_frames, self.segment_id
        )?;
        writeln!(
            f,
            "driver         pid {}, rate {}, epoch {}, running {}, loopback {}, cycle {}, \
             heartbeat {}",
            self.driver_pid,
            self.sample_rate,
            self.io_epoch,
            self.io_running as u8,
            self.loopback_active as u8,
            self.cycle_seq,
            age(self.driver_heartbeat_age_ns)
        )?;
        writeln!(
            f,
            "input          input_time {}, input_frames {}",
            self.input_time, self.input_frames
        )?;
        writeln!(
            f,
            "daemon         pid {}, heartbeat {}",
            self.daemon_pid,
            age(self.daemon_heartbeat_age_ns)
        )?;
        for (n, r) in [
            ("to_daemon", &self.to_daemon),
            ("from_daemon", &self.from_daemon),
        ] {
            writeln!(
                f,
                "{n:<14} write_time {}, read_time {}, epoch {}, overruns {}, underruns {}",
                r.write_time, r.read_time, r.epoch, r.overruns, r.underruns
            )?;
        }
        Ok(())
    }
}
