//! Minimal RIFF/WAVE reader and writer (no dependencies), shared by the daemon's recorder (T061)
//! and `wing-render` (T060).
//!
//! * [`WavWriter`]: 32-bit IEEE float, any channel count (`WAVE_FORMAT_IEEE_FLOAT` for 1–2
//!   channels, `WAVE_FORMAT_EXTENSIBLE` above), streaming; sizes are patched by
//!   [`WavWriter::finish`] (and on drop, best effort). Files over 4 GiB are not supported (RIFF
//!   limit): writing stops growing the header at that point.
//! * [`WavReader`]: streaming reader of PCM 16/24/32-bit integer and 32/64-bit float, plain or
//!   `WAVE_FORMAT_EXTENSIBLE`, any channel count; samples come out as `f32` in [-1, 1).
//!   RF64/BW64 and compressed formats are rejected.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;

const FMT_PCM: u16 = 1;
const FMT_FLOAT: u16 = 3;
const FMT_EXTENSIBLE: u16 = 0xFFFE;

fn bad(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

// ---- writer -------------------------------------------------------------------------------------

/// Streaming 32-bit float WAV writer.
pub struct WavWriter {
    out: Option<BufWriter<File>>,
    channels: u16,
    /// Offset of the `data` chunk size field.
    data_size_at: u64,
    data_bytes: u64,
    frames: u64,
}

impl WavWriter {
    /// Create `path` (truncating) for `channels` channels at `sample_rate`.
    pub fn create(path: &Path, channels: u16, sample_rate: u32) -> io::Result<Self> {
        if channels == 0 {
            return Err(bad("WAV needs at least one channel"));
        }
        let mut out = BufWriter::with_capacity(1 << 16, File::create(path)?);
        let ext = channels > 2;
        let block_align = channels as u32 * 4;
        let fmt_len: u32 = if ext { 40 } else { 18 };
        let mut h = Vec::with_capacity(80);
        h.extend_from_slice(b"RIFF");
        h.extend_from_slice(&0u32.to_le_bytes()); // patched
        h.extend_from_slice(b"WAVE");
        h.extend_from_slice(b"fmt ");
        h.extend_from_slice(&fmt_len.to_le_bytes());
        h.extend_from_slice(&(if ext { FMT_EXTENSIBLE } else { FMT_FLOAT }).to_le_bytes());
        h.extend_from_slice(&channels.to_le_bytes());
        h.extend_from_slice(&sample_rate.to_le_bytes());
        h.extend_from_slice(&(sample_rate * block_align).to_le_bytes());
        h.extend_from_slice(&(block_align as u16).to_le_bytes());
        h.extend_from_slice(&32u16.to_le_bytes());
        if ext {
            h.extend_from_slice(&22u16.to_le_bytes()); // cbSize
            h.extend_from_slice(&32u16.to_le_bytes()); // valid bits
            h.extend_from_slice(&0u32.to_le_bytes()); // channel mask: unassigned
                                                      // KSDATAFORMAT_SUBTYPE_IEEE_FLOAT {00000003-0000-0010-8000-00aa00389b71}
            h.extend_from_slice(&[
                0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38,
                0x9b, 0x71,
            ]);
        } else {
            h.extend_from_slice(&0u16.to_le_bytes()); // cbSize
        }
        // `fact` chunk (required for non-PCM by the spec; frames patched).
        h.extend_from_slice(b"fact");
        h.extend_from_slice(&4u32.to_le_bytes());
        h.extend_from_slice(&0u32.to_le_bytes());
        h.extend_from_slice(b"data");
        let data_size_at = h.len() as u64;
        h.extend_from_slice(&0u32.to_le_bytes());
        out.write_all(&h)?;
        Ok(Self {
            out: Some(out),
            channels,
            data_size_at,
            data_bytes: 0,
            frames: 0,
        })
    }

    pub fn channels(&self) -> u16 {
        self.channels
    }

    /// Frames written so far.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Append interleaved samples (a whole number of frames).
    pub fn write_interleaved(&mut self, samples: &[f32]) -> io::Result<()> {
        let out = self.out.as_mut().ok_or_else(|| bad("writer finished"))?;
        debug_assert_eq!(samples.len() % self.channels as usize, 0);
        let mut buf = [0u8; 4096];
        for chunk in samples.chunks(1024) {
            for (i, s) in chunk.iter().enumerate() {
                buf[i * 4..i * 4 + 4].copy_from_slice(&s.to_le_bytes());
            }
            out.write_all(&buf[..chunk.len() * 4])?;
        }
        self.data_bytes += samples.len() as u64 * 4;
        self.frames += (samples.len() / self.channels as usize) as u64;
        Ok(())
    }

    /// Append planar samples: `planes[c][i]` (all planes the same length).
    pub fn write_planar(&mut self, planes: &[&[f32]]) -> io::Result<()> {
        let n = planes.iter().map(|p| p.len()).min().unwrap_or(0);
        let mut inter = Vec::with_capacity(n * planes.len());
        for i in 0..n {
            for p in planes {
                inter.push(p[i]);
            }
        }
        self.write_interleaved(&inter)
    }

    fn patch(&mut self) -> io::Result<()> {
        let Some(out) = self.out.as_mut() else {
            return Ok(());
        };
        out.flush()?;
        let f = out.get_mut();
        let data = self.data_bytes.min(u32::MAX as u64 - 64) as u32;
        let riff = (self.data_size_at as u32 + 4 - 8).saturating_add(data);
        f.seek(SeekFrom::Start(4))?;
        f.write_all(&riff.to_le_bytes())?;
        // fact sample count sits 8 bytes before the data chunk header
        f.seek(SeekFrom::Start(self.data_size_at - 8))?;
        f.write_all(&(self.frames.min(u32::MAX as u64) as u32).to_le_bytes())?;
        f.seek(SeekFrom::Start(self.data_size_at))?;
        f.write_all(&data.to_le_bytes())?;
        f.seek(SeekFrom::End(0))?;
        f.flush()
    }

    /// Patch the header sizes (the file stays valid after every call) .
    pub fn flush_header(&mut self) -> io::Result<()> {
        self.patch()
    }

    /// Patch the header and close the file.
    pub fn finish(mut self) -> io::Result<u64> {
        self.patch()?;
        self.out = None;
        Ok(self.frames)
    }
}

impl Drop for WavWriter {
    fn drop(&mut self) {
        let _ = self.patch();
    }
}

// ---- reader -------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Encoding {
    Int(u16),
    Float(u16),
}

/// Format of a WAV file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WavInfo {
    pub channels: u16,
    pub sample_rate: u32,
    pub bits: u16,
    pub float: bool,
    /// Frames in the data chunk.
    pub frames: u64,
}

/// Streaming WAV reader (see the module docs for the supported formats).
pub struct WavReader {
    r: BufReader<File>,
    info: WavInfo,
    enc: Encoding,
    frame_bytes: usize,
    data_start: u64,
    pos: u64,
    raw: Vec<u8>,
}

impl WavReader {
    pub fn open(path: &Path) -> io::Result<Self> {
        let mut r = BufReader::with_capacity(1 << 16, File::open(path)?);
        let mut hdr = [0u8; 12];
        r.read_exact(&mut hdr)?;
        if &hdr[0..4] != b"RIFF" || &hdr[8..12] != b"WAVE" {
            if &hdr[0..4] == b"RF64" || &hdr[0..4] == b"BW64" {
                return Err(bad("RF64/BW64 WAV files are not supported"));
            }
            return Err(bad("not a RIFF/WAVE file"));
        }
        let mut fmt: Option<(u16, u16, u32, u16, u16)> = None;
        let mut pos = 12u64;
        loop {
            let mut ch = [0u8; 8];
            if r.read_exact(&mut ch).is_err() {
                return Err(bad("no data chunk"));
            }
            pos += 8;
            let id = [ch[0], ch[1], ch[2], ch[3]];
            let len = u32::from_le_bytes([ch[4], ch[5], ch[6], ch[7]]) as u64;
            match &id {
                b"fmt " => {
                    let mut b = vec![0u8; len as usize];
                    r.read_exact(&mut b)?;
                    if b.len() < 16 {
                        return Err(bad("short fmt chunk"));
                    }
                    let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
                    let mut tag = u16_at(0);
                    let channels = u16_at(2);
                    let rate = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                    let align = u16_at(12);
                    let bits = u16_at(14);
                    if tag == FMT_EXTENSIBLE {
                        if b.len() < 40 {
                            return Err(bad("short WAVE_FORMAT_EXTENSIBLE fmt chunk"));
                        }
                        tag = u16_at(24);
                    }
                    fmt = Some((tag, channels, rate, align, bits));
                    pos += len;
                    if len % 2 == 1 {
                        r.seek_relative(1)?;
                        pos += 1;
                    }
                }
                b"data" => {
                    let (tag, channels, sample_rate, align, bits) =
                        fmt.ok_or_else(|| bad("data chunk before fmt chunk"))?;
                    let enc = match (tag, bits) {
                        (FMT_PCM, 8 | 16 | 24 | 32) => Encoding::Int(bits),
                        (FMT_FLOAT, 32 | 64) => Encoding::Float(bits),
                        _ => {
                            return Err(bad(format!(
                                "unsupported WAV encoding (format tag {tag}, {bits} bits)"
                            )))
                        }
                    };
                    if channels == 0 || align as usize != channels as usize * (bits as usize / 8) {
                        return Err(bad("inconsistent WAV block alignment"));
                    }
                    let frame_bytes = align as usize;
                    let flen = r.get_ref().metadata()?.len();
                    // Some writers leave the size at 0/0xFFFFFFFF while streaming: use the file.
                    let len = if len == 0 || len == u32::MAX as u64 || pos + len > flen {
                        flen.saturating_sub(pos)
                    } else {
                        len
                    };
                    let info = WavInfo {
                        channels,
                        sample_rate,
                        bits,
                        float: matches!(enc, Encoding::Float(_)),
                        frames: len / frame_bytes as u64,
                    };
                    return Ok(Self {
                        r,
                        info,
                        enc,
                        frame_bytes,
                        data_start: pos,
                        pos: 0,
                        raw: Vec::new(),
                    });
                }
                _ => {
                    let skip = len + (len & 1);
                    r.seek_relative(skip as i64)?;
                    pos += skip;
                }
            }
        }
    }

    pub fn info(&self) -> WavInfo {
        self.info
    }

    /// Frames left to read.
    pub fn remaining(&self) -> u64 {
        self.info.frames - self.pos
    }

    /// Seek to frame `frame` (clamped to the end).
    pub fn seek(&mut self, frame: u64) -> io::Result<()> {
        let f = frame.min(self.info.frames);
        if f == self.pos {
            return Ok(()); // keep the read buffer
        }
        self.r.seek(SeekFrom::Start(
            self.data_start + f * self.frame_bytes as u64,
        ))?;
        self.pos = f;
        Ok(())
    }

    /// Read up to `out.len() / channels` frames, interleaved, into `out`. Returns the frames
    /// read (0 at the end).
    pub fn read_interleaved(&mut self, out: &mut [f32]) -> io::Result<usize> {
        let ch = self.info.channels as usize;
        let frames = ((out.len() / ch) as u64).min(self.remaining()) as usize;
        if frames == 0 {
            return Ok(0);
        }
        let nbytes = frames * self.frame_bytes;
        self.raw.resize(nbytes, 0);
        self.r.read_exact(&mut self.raw[..nbytes])?;
        let raw = &self.raw[..nbytes];
        let out = &mut out[..frames * ch];
        match self.enc {
            Encoding::Int(8) => {
                for (o, b) in out.iter_mut().zip(raw) {
                    *o = (*b as f32 - 128.0) / 128.0;
                }
            }
            Encoding::Int(16) => {
                for (o, b) in out.iter_mut().zip(raw.as_chunks::<2>().0) {
                    *o = i16::from_le_bytes([b[0], b[1]]) as f32 / 32_768.0;
                }
            }
            Encoding::Int(24) => {
                for (o, b) in out.iter_mut().zip(raw.as_chunks::<3>().0) {
                    let v = i32::from_le_bytes([0, b[0], b[1], b[2]]) >> 8;
                    *o = v as f32 / 8_388_608.0;
                }
            }
            Encoding::Int(_) => {
                for (o, b) in out.iter_mut().zip(raw.as_chunks::<4>().0) {
                    let v = i32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                    *o = (v as f64 / 2_147_483_648.0) as f32;
                }
            }
            Encoding::Float(32) => {
                for (o, b) in out.iter_mut().zip(raw.as_chunks::<4>().0) {
                    *o = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                }
            }
            Encoding::Float(_) => {
                for (o, b) in out.iter_mut().zip(raw.as_chunks::<8>().0) {
                    *o = f64::from_le_bytes(*b) as f32;
                }
            }
        }
        self.pos += frames as u64;
        Ok(frames)
    }
}

/// Read a whole file: (info, interleaved samples).
pub fn read_all(path: &Path) -> io::Result<(WavInfo, Vec<f32>)> {
    let mut r = WavReader::open(path)?;
    let info = r.info();
    let mut v = vec![0.0f32; info.frames as usize * info.channels as usize];
    let mut done = 0;
    while done < v.len() {
        let n = r.read_interleaved(&mut v[done..])?;
        if n == 0 {
            break;
        }
        done += n * info.channels as usize;
    }
    v.truncate(done);
    Ok((info, v))
}

/// Write a PCM integer WAV (16/24/32 bit) from interleaved `f32` (test fixtures, tools).
pub fn write_pcm(
    path: &Path,
    channels: u16,
    sample_rate: u32,
    bits: u16,
    samples: &[f32],
) -> io::Result<()> {
    if !matches!(bits, 16 | 24 | 32) {
        return Err(bad("PCM bits must be 16, 24 or 32"));
    }
    let bps = bits as u32 / 8;
    let data = samples.len() as u32 * bps;
    let mut w = BufWriter::new(File::create(path)?);
    w.write_all(b"RIFF")?;
    w.write_all(&(36 + data).to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&FMT_PCM.to_le_bytes())?;
    w.write_all(&channels.to_le_bytes())?;
    w.write_all(&sample_rate.to_le_bytes())?;
    w.write_all(&(sample_rate * channels as u32 * bps).to_le_bytes())?;
    w.write_all(&((channels as u32 * bps) as u16).to_le_bytes())?;
    w.write_all(&bits.to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data.to_le_bytes())?;
    for &s in samples {
        let full = (1i64 << (bits - 1)) as f64;
        let v = (s as f64 * full).round().clamp(-full, full - 1.0) as i64;
        w.write_all(&v.to_le_bytes()[..bps as usize])?;
    }
    w.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("wingsim-wav-{}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d.join(name)
    }

    #[test]
    fn float_round_trip_mono_stereo_multi() {
        for ch in [1u16, 2, 6] {
            let p = tmp(&format!("f{ch}.wav"));
            let s: Vec<f32> = (0..ch as usize * 1000)
                .map(|i| ((i as f32) * 0.37).sin() * 0.9)
                .collect();
            let mut w = WavWriter::create(&p, ch, 48_000).unwrap();
            w.write_interleaved(&s[..ch as usize * 300]).unwrap();
            w.write_interleaved(&s[ch as usize * 300..]).unwrap();
            assert_eq!(w.finish().unwrap(), 1000);
            let (info, back) = read_all(&p).unwrap();
            assert_eq!(info.channels, ch);
            assert_eq!(info.sample_rate, 48_000);
            assert!(info.float && info.bits == 32);
            assert_eq!(back, s);
        }
    }

    #[test]
    fn pcm_24_and_32_and_16_read() {
        let s: Vec<f32> = (0..2000).map(|i| ((i as f32) * 0.01).sin() * 0.5).collect();
        for (bits, tol) in [(16u16, 1.0 / 32768.0), (24, 1.0 / 8e6), (32, 1e-7)] {
            let p = tmp(&format!("p{bits}.wav"));
            write_pcm(&p, 2, 44_100, bits, &s).unwrap();
            let mut r = WavReader::open(&p).unwrap();
            assert_eq!(r.info().frames, 1000);
            assert_eq!(r.info().bits, bits);
            let mut buf = vec![0.0; 2 * 333];
            let mut all = Vec::new();
            loop {
                let n = r.read_interleaved(&mut buf).unwrap();
                if n == 0 {
                    break;
                }
                all.extend_from_slice(&buf[..n * 2]);
            }
            assert_eq!(all.len(), s.len());
            for (a, b) in all.iter().zip(&s) {
                assert!((a - b).abs() <= tol as f32 * 1.01, "{bits}: {a} vs {b}");
            }
            r.seek(990).unwrap();
            assert_eq!(r.read_interleaved(&mut buf).unwrap(), 10);
        }
    }

    #[test]
    fn rejects_garbage() {
        let p = tmp("junk.wav");
        std::fs::write(&p, b"not a wave file at all").unwrap();
        assert!(WavReader::open(&p).is_err());
    }
}
