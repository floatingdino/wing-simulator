//! CoreAudio (HAL) side of the bridge: device enumeration and the IO-proc output.

mod device;
mod output;
mod sys;

pub use device::{
    buffer_frames, find_output_device, host_ticks_to_ns, host_time_ns, is_alive, list_devices,
    list_output_devices, nominal_rate, output_latency_frames, CaError, DeviceInfo, DeviceWorkgroup,
};
pub use output::{HostOutput, HostOutputConfig};
pub use sys::AudioObjectID;
