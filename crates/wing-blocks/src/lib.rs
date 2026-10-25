//! WING processing blocks and alternate models (filter, gate, EQ, dynamics, FX).
//!
//! Every block is a plain struct with a typed parameter struct in engineering units, `prepare(sample_rate,
//! max_block)` (the only allocating call), `set_params` (RT-safe) and an in-place stereo `process`.

pub mod filter;
pub mod gate;
pub mod input;
pub mod sidechain;
pub mod wingcoef;

pub mod dynamics;
pub mod eq;
pub mod fidelity;
pub mod meter;
pub mod model;
pub mod models;
pub mod std_models;

#[cfg(test)]
mod tests {
    #[test]
    fn scaffold() {
        assert_eq!(1 + 1, 2);
    }
}
