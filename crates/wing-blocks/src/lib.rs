//! WING processing blocks and alternate models (filter, gate, EQ, dynamics, FX).
//!
//! Every block is a plain struct with a typed parameter struct in engineering units, `prepare(sample_rate,
//! max_block)` (the only allocating call), `set_params` (RT-safe) and an in-place stereo `process`.

pub mod filter;
pub mod gate;
pub mod input;

pub mod dynamics;
pub mod eq;

#[cfg(test)]
mod tests {
    #[test]
    fn scaffold() {
        assert_eq!(1 + 1, 2);
    }
}
