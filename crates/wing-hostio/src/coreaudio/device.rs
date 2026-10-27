//! Device enumeration and property access (non-RT).

use std::ffi::c_void;
use std::fmt;
use std::mem::{size_of, MaybeUninit};

use wing_engine::RawWorkgroup;

use super::sys::*;

/// CoreAudio error with context.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaError {
    /// What was being done.
    pub what: String,
    /// OSStatus (0 for non-HAL errors).
    pub status: OSStatus,
}

impl fmt::Display for CaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.status == 0 {
            write!(f, "{}", self.what)
        } else {
            write!(f, "{}: {}", self.what, status_str(self.status))
        }
    }
}

impl std::error::Error for CaError {}

pub(crate) fn check(st: OSStatus, what: impl FnOnce() -> String) -> Result<(), CaError> {
    if st == 0 {
        Ok(())
    } else {
        Err(CaError {
            what: what(),
            status: st,
        })
    }
}

pub(crate) fn addr(sel: u32, scope: u32) -> AudioObjectPropertyAddress {
    AudioObjectPropertyAddress {
        mSelector: sel,
        mScope: scope,
        mElement: kAudioObjectPropertyElementMain,
    }
}

fn get<T: Copy>(obj: AudioObjectID, sel: u32, scope: u32) -> Result<T, CaError> {
    let a = addr(sel, scope);
    let mut v = MaybeUninit::<T>::zeroed();
    let mut size = size_of::<T>() as u32;
    // SAFETY: `v` has room for `size` bytes; T is plain data.
    let st = unsafe {
        AudioObjectGetPropertyData(
            obj,
            &a,
            0,
            std::ptr::null(),
            &mut size,
            v.as_mut_ptr() as *mut c_void,
        )
    };
    check(st, || {
        format!("get property {} of object {obj}", status_str(sel as i32))
    })?;
    // SAFETY: zero-initialised and (partially) written by the HAL; T is plain data.
    Ok(unsafe { v.assume_init() })
}

fn get_bytes(obj: AudioObjectID, sel: u32, scope: u32) -> Result<Vec<u8>, CaError> {
    let a = addr(sel, scope);
    let mut size = 0u32;
    // SAFETY: plain FFI with valid pointers.
    let st = unsafe { AudioObjectGetPropertyDataSize(obj, &a, 0, std::ptr::null(), &mut size) };
    check(st, || {
        format!("size of {} on {obj}", status_str(sel as i32))
    })?;
    // u64-backed buffer so variable-length structs are suitably aligned.
    let mut buf = vec![0u64; (size as usize).div_ceil(8)];
    // SAFETY: buffer is at least `size` bytes.
    let st = unsafe {
        AudioObjectGetPropertyData(
            obj,
            &a,
            0,
            std::ptr::null(),
            &mut size,
            buf.as_mut_ptr() as *mut c_void,
        )
    };
    check(st, || format!("get {} on {obj}", status_str(sel as i32)))?;
    // SAFETY: reinterpreting initialised u64s as bytes.
    let bytes = unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, size as usize) };
    Ok(bytes.to_vec())
}

fn get_string(obj: AudioObjectID, sel: u32) -> Result<String, CaError> {
    let s: CFStringRef = get(obj, sel, kAudioObjectPropertyScopeGlobal)?;
    if s.is_null() {
        return Ok(String::new());
    }
    // SAFETY: `s` is a retained CFString we own (Copy rule for HAL string properties).
    unsafe {
        let len = CFStringGetLength(s);
        let max = CFStringGetMaximumSizeForEncoding(len, kCFStringEncodingUTF8) + 1;
        let mut buf = vec![0u8; max.max(1) as usize];
        let ok = CFStringGetCString(s, buf.as_mut_ptr() as *mut _, max, kCFStringEncodingUTF8);
        CFRelease(s);
        if ok == 0 {
            return Ok(String::new());
        }
        let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        Ok(String::from_utf8_lossy(&buf[..end]).into_owned())
    }
}

/// Per-buffer channel counts of a device's stream configuration in `scope`.
pub(crate) fn stream_layout(dev: AudioObjectID, scope: u32) -> Result<Vec<u32>, CaError> {
    let bytes = get_bytes(dev, kAudioDevicePropertyStreamConfiguration, scope)?;
    if bytes.len() < 4 {
        return Ok(Vec::new());
    }
    let n = u32::from_ne_bytes(bytes[0..4].try_into().unwrap()) as usize;
    let mut out = Vec::with_capacity(n);
    // AudioBufferList: u32 count, 4 bytes padding, then 16-byte AudioBuffers.
    let off = std::mem::offset_of!(AudioBufferList, mBuffers);
    for i in 0..n {
        let o = off + i * size_of::<AudioBuffer>();
        if o + 4 > bytes.len() {
            break;
        }
        out.push(u32::from_ne_bytes(bytes[o..o + 4].try_into().unwrap()));
    }
    Ok(out)
}

/// Basic facts about a device.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceInfo {
    /// HAL object id (changes across reconnects).
    pub id: AudioObjectID,
    /// Display name.
    pub name: String,
    /// Persistent UID.
    pub uid: String,
    /// Output channel count (all output streams).
    pub output_channels: u32,
    /// Input channel count.
    pub input_channels: u32,
    /// Nominal sample rate.
    pub sample_rate: f64,
    /// IO buffer size in frames.
    pub buffer_frames: u32,
}

impl DeviceInfo {
    /// Query a device by id.
    pub fn query(id: AudioObjectID) -> Result<Self, CaError> {
        let out: u32 = stream_layout(id, kAudioObjectPropertyScopeOutput)?
            .iter()
            .sum();
        let inp: u32 = stream_layout(id, kAudioObjectPropertyScopeInput)
            .unwrap_or_default()
            .iter()
            .sum();
        Ok(Self {
            id,
            name: get_string(id, kAudioObjectPropertyName)?,
            uid: get_string(id, kAudioDevicePropertyDeviceUID).unwrap_or_default(),
            output_channels: out,
            input_channels: inp,
            sample_rate: get(
                id,
                kAudioDevicePropertyNominalSampleRate,
                kAudioObjectPropertyScopeGlobal,
            )?,
            buffer_frames: get(
                id,
                kAudioDevicePropertyBufferFrameSize,
                kAudioObjectPropertyScopeGlobal,
            )
            .unwrap_or(0),
        })
    }
}

/// All audio device ids.
pub fn device_ids() -> Result<Vec<AudioObjectID>, CaError> {
    let bytes = get_bytes(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
    )?;
    Ok(bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| u32::from_ne_bytes(*c))
        .collect())
}

/// All devices (devices that fail to answer are skipped).
pub fn list_devices() -> Result<Vec<DeviceInfo>, CaError> {
    Ok(device_ids()?
        .into_iter()
        .filter_map(|id| DeviceInfo::query(id).ok())
        .collect())
}

/// Devices with at least one output channel.
pub fn list_output_devices() -> Result<Vec<DeviceInfo>, CaError> {
    Ok(list_devices()?
        .into_iter()
        .filter(|d| d.output_channels > 0)
        .collect())
}

/// The system default output device, or, if that is the simulator's own device, the first
/// other output device (routing the simulator into itself would feed back).
pub fn default_output_device() -> Result<Option<DeviceInfo>, CaError> {
    let id: AudioObjectID = get(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyDefaultOutputDevice,
        kAudioObjectPropertyScopeGlobal,
    )?;
    let devs = list_output_devices()?;
    let usable = |d: &&DeviceInfo| !d.name.eq_ignore_ascii_case(SIMULATOR_DEVICE);
    Ok(devs
        .iter()
        .filter(usable)
        .find(|d| d.id == id)
        .or_else(|| devs.iter().find(usable))
        .cloned())
}

/// Name of the simulator's own (driver) device.
const SIMULATOR_DEVICE: &str = "Wing Simulator";

/// Find an output device: [`crate::DEFAULT_OUTPUT`] is [`default_output_device`]; otherwise by
/// exact UID, exact name, then case-insensitive name, then case-insensitive substring of the name.
pub fn find_output_device(key: &str) -> Result<Option<DeviceInfo>, CaError> {
    if key == crate::DEFAULT_OUTPUT {
        return default_output_device();
    }
    let devs = list_output_devices()?;
    let lower = key.to_lowercase();
    let pick = devs
        .iter()
        .position(|d| d.uid == key)
        .or_else(|| devs.iter().position(|d| d.name == key))
        .or_else(|| devs.iter().position(|d| d.name.to_lowercase() == lower))
        .or_else(|| {
            devs.iter()
                .position(|d| d.name.to_lowercase().contains(&lower))
        });
    Ok(pick.map(|i| devs[i].clone()))
}

/// Whether the device still exists and is alive.
pub fn is_alive(id: AudioObjectID) -> bool {
    get::<u32>(
        id,
        kAudioDevicePropertyDeviceIsAlive,
        kAudioObjectPropertyScopeGlobal,
    )
    .map(|v| v != 0)
    .unwrap_or(false)
}

/// Current nominal sample rate.
pub fn nominal_rate(id: AudioObjectID) -> Result<f64, CaError> {
    get(
        id,
        kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal,
    )
}

/// Current IO buffer size.
pub fn buffer_frames(id: AudioObjectID) -> Result<u32, CaError> {
    get(
        id,
        kAudioDevicePropertyBufferFrameSize,
        kAudioObjectPropertyScopeGlobal,
    )
}

/// Request an IO buffer size (applies to this process; the HAL may round it).
pub fn set_buffer_frames(id: AudioObjectID, frames: u32) -> Result<(), CaError> {
    let a = addr(
        kAudioDevicePropertyBufferFrameSize,
        kAudioObjectPropertyScopeGlobal,
    );
    // SAFETY: plain FFI with a u32 payload.
    let st = unsafe {
        AudioObjectSetPropertyData(id, &a, 0, std::ptr::null(), 4, &frames as *const u32 as _)
    };
    check(st, || format!("set buffer size {frames} on device {id}"))
}

/// Output latency in device frames (device latency + safety offset), best effort.
pub fn output_latency_frames(id: AudioObjectID) -> u32 {
    let l: u32 = get(
        id,
        kAudioDevicePropertyLatency,
        kAudioObjectPropertyScopeOutput,
    )
    .unwrap_or(0);
    let s: u32 = get(
        id,
        kAudioDevicePropertySafetyOffset,
        kAudioObjectPropertyScopeOutput,
    )
    .unwrap_or(0);
    l + s
}

/// Whether every output stream's virtual format is 32-bit float linear PCM (what the IO proc
/// writes).
pub fn outputs_are_f32(id: AudioObjectID) -> Result<bool, CaError> {
    let bytes = get_bytes(
        id,
        kAudioDevicePropertyStreams,
        kAudioObjectPropertyScopeOutput,
    )?;
    for c in bytes.as_chunks::<4>().0 {
        let stream = u32::from_ne_bytes(*c);
        let f: AudioStreamBasicDescription = get(
            stream,
            kAudioStreamPropertyVirtualFormat,
            kAudioObjectPropertyScopeGlobal,
        )?;
        if f.mFormatID != kAudioFormatLinearPCM
            || f.mFormatFlags & kAudioFormatFlagIsFloat == 0
            || f.mBitsPerChannel != 32
        {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Tell the HAL to deliver property notifications on its own thread instead of the main run
/// loop (a daemon has no main run loop). Process-wide; idempotent.
pub fn use_hal_notification_thread() {
    let a = addr(
        kAudioHardwarePropertyRunLoop,
        kAudioObjectPropertyScopeGlobal,
    );
    let rl: *const c_void = std::ptr::null();
    // SAFETY: the property takes a CFRunLoopRef; NULL means "HAL-owned thread".
    unsafe {
        AudioObjectSetPropertyData(
            kAudioObjectSystemObject,
            &a,
            0,
            std::ptr::null(),
            size_of::<*const c_void>() as u32,
            &rl as *const _ as *const c_void,
        );
    }
}

/// A retained `os_workgroup_t` of a device's IO thread; released on drop.
pub struct DeviceWorkgroup(*mut c_void);

// SAFETY: os_workgroup objects are thread-safe; we only hold a retain.
unsafe impl Send for DeviceWorkgroup {}
// SAFETY: as above.
unsafe impl Sync for DeviceWorkgroup {}

impl DeviceWorkgroup {
    /// The device's IO workgroup (macOS 11+). Threads doing work on behalf of that device's
    /// IO cycle can join it with [`wing_engine::WorkgroupMembership::join`].
    pub fn of_device(id: AudioObjectID) -> Option<Self> {
        let p: *mut c_void = get(
            id,
            kAudioDevicePropertyIOThreadOSWorkgroup,
            kAudioObjectPropertyScopeGlobal,
        )
        .ok()?;
        (!p.is_null()).then_some(Self(p))
    }

    /// As a [`RawWorkgroup`] (valid while `self` lives).
    pub fn raw(&self) -> RawWorkgroup {
        // SAFETY: we hold a retain for our lifetime.
        unsafe { RawWorkgroup::from_raw(self.0) }
    }
}

impl Drop for DeviceWorkgroup {
    fn drop(&mut self) {
        // SAFETY: balanced with the retain the HAL gave us.
        unsafe { os_release(self.0) }
    }
}

/// Host time now, in nanoseconds (same clock as `AudioTimeStamp::mHostTime`).
pub fn host_time_ns() -> u64 {
    // SAFETY: trivial FFI.
    host_ticks_to_ns(unsafe { mach_absolute_time() })
}

/// Convert `mach_absolute_time` ticks to nanoseconds.
#[inline]
pub fn host_ticks_to_ns(ticks: u64) -> u64 {
    let (n, d) = timebase();
    (ticks as u128 * n as u128 / d as u128) as u64
}

fn timebase() -> (u32, u32) {
    use std::sync::OnceLock;
    static TB: OnceLock<(u32, u32)> = OnceLock::new();
    *TB.get_or_init(|| {
        #[repr(C)]
        struct Tb {
            numer: u32,
            denom: u32,
        }
        extern "C" {
            fn mach_timebase_info(info: *mut Tb) -> i32;
        }
        let mut tb = Tb { numer: 0, denom: 0 };
        // SAFETY: writes into a valid struct.
        let r = unsafe { mach_timebase_info(&mut tb) };
        if r != 0 || tb.numer == 0 || tb.denom == 0 {
            (1, 1)
        } else {
            (tb.numer, tb.denom)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enumerate_does_not_fail() {
        // Headless CI machines may have no devices; just exercise the calls.
        let devs = list_devices().unwrap();
        for d in &devs {
            assert!(d.sample_rate >= 0.0);
        }
        let _ = find_output_device("definitely-not-a-device-name-xyz").unwrap();
        if let Some(d) = find_output_device(crate::DEFAULT_OUTPUT).unwrap() {
            assert_ne!(d.name, SIMULATOR_DEVICE);
            assert!(d.output_channels > 0);
        }
    }

    #[test]
    fn host_time_monotonic() {
        let a = host_time_ns();
        let b = host_time_ns();
        assert!(b >= a);
        assert!(a > 0);
    }
}
