//! WING node hash (bit-exact port of `tools/schema-extract/winghash.py`).
//!
//! The console identifies every node by a 32-bit hash of its path, computed by
//! `ncl::Cnode::setRoot`: a MurmurHash3-x86-32 variant over the `getPath()` text without its
//! first character (`ch.1.flt.lcf`). ASCII `a-z` is upper-cased, spaces are skipped, four
//! characters are packed big-endian per block, a trailing partial block gets the full block mix,
//! and the result is `h ^ nchars` with no final avalanche. See docs/research/wing-edit-schema.md §3.

const SEED: u32 = 0xCB21_C6BE;

#[inline(always)]
const fn mix(h: u32, k: u32) -> u32 {
    let k = k
        .wrapping_mul(0xCC9E_2D51)
        .rotate_left(15)
        .wrapping_mul(0x1B87_3593);
    (h ^ k)
        .rotate_left(13)
        .wrapping_mul(5)
        .wrapping_add(0xE654_6B64)
}

/// Hash a node path. Accepts the slash form (`ch/1/flt/lcf`, `/ch/1/flt/lcf`) or the `getPath()`
/// form (`/ch.1.flt.lcf`); `/` is hashed as `.`. The empty path (root) hashes to 0.
#[inline]
pub fn wing_hash(path: &str) -> u32 {
    let mut b = path.as_bytes();
    while let [b'/', rest @ ..] = b {
        b = rest;
    }
    while let [rest @ .., b'/'] = b {
        b = rest;
    }
    if b.is_empty() {
        return 0;
    }
    let mut h = SEED;
    let mut k = 0u32;
    let mut cnt = 0u32;
    let mut total = 0u32;
    for &c in b {
        let c = match c {
            b'/' => b'.',
            b'a'..=b'z' => c - 0x20,
            b' ' => continue,
            _ => c,
        };
        k = (k << 8) | c as u32;
        cnt += 1;
        total += 1;
        if cnt == 4 {
            h = mix(h, k);
            k = 0;
            cnt = 0;
        }
    }
    if cnt != 0 || total == 0 {
        h = mix(h, k);
    }
    h ^ total
}

#[cfg(test)]
mod tests {
    use super::wing_hash;

    #[test]
    fn documented_hashes() {
        // docs/research/protocol.md and wing-edit-schema.md
        assert_eq!(wing_hash("/ch/1/mute"), 0xF50F_69F8);
        assert_eq!(wing_hash("ch/1/fdr"), 0x38AE_75C2);
        assert_eq!(wing_hash("/ch.1.fdr"), 0x38AE_75C2);
        assert_eq!(wing_hash("ch/1/dyn/1"), 0x104B_1346);
        assert_eq!(wing_hash(""), 0);
        assert_eq!(wing_hash("/"), 0);
    }

    #[test]
    fn case_and_space_insensitive() {
        assert_eq!(wing_hash("CH/1/FDR"), wing_hash("ch/1/fdr"));
        assert_eq!(wing_hash("ch/1/f dr"), wing_hash("ch/1/fdr"));
    }
}
