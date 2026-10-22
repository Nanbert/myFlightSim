//! The 0–10 row scorer behind every graded card.
//!
//! Each row grades one deviation against two anchors: within `tol` is a
//! perfect 10, at `cap` is a zero, and everything between falls off
//! linearly. A row reports its number **and a word** (TIGHT / GOOD / OK /
//! SLOPPY / BAD) on shared bands, so any future scorer plugs into the same
//! language: numbers, with the words kept for reading at a glance.
//!
//! The lesson finals are a weighted average over their rows — a bad row
//! pulls the final down, it never zeroes the card, and no worst-row clamp
//! sits on top (deliberate: the graded rows are all recoverable; the flat
//! FAILs are handled as popups, outside this scorer).

/// A row's 0–10 score: `err <= tol` is a perfect ten, `err >= cap` is a
/// zero, and everything between falls off linearly.
pub fn score(err: f32, tol: f32, cap: f32) -> f32 {
    if err <= tol {
        10.0
    } else if err >= cap {
        0.0
    } else {
        10.0 * (cap - err) / (cap - tol)
    }
}

/// The word bands shared by every row and every lesson's final.
pub fn word(score: f32) -> &'static str {
    if score >= 9.0 {
        "TIGHT"
    } else if score >= 7.5 {
        "GOOD"
    } else if score >= 6.0 {
        "OK"
    } else if score >= 3.5 {
        "SLOPPY"
    } else {
        "BAD"
    }
}

#[cfg(test)]
mod tests {
    use super::{score, word};

    const TOL: f32 = 1.0;
    const CAP: f32 = 8.0;

    #[test]
    fn within_the_tolerance_is_a_perfect_ten() {
        assert_eq!(score(0.0, TOL, CAP), 10.0);
        assert_eq!(score(TOL, TOL, CAP), 10.0);
    }

    #[test]
    fn at_the_cap_is_a_zero_and_beyond_stays_there() {
        assert_eq!(score(CAP, TOL, CAP), 0.0);
        assert_eq!(score(CAP * 2.0, TOL, CAP), 0.0);
    }

    #[test]
    fn the_middle_is_linear() {
        // Halfway between tol and cap is exactly half marks.
        let mid = (TOL + CAP) / 2.0;
        assert!((score(mid, TOL, CAP) - 5.0).abs() < 1e-4);
    }

    #[test]
    fn the_words_band_the_numbers() {
        assert_eq!(word(10.0), "TIGHT");
        assert_eq!(word(9.0), "TIGHT");
        assert_eq!(word(8.9), "GOOD");
        assert_eq!(word(7.5), "GOOD");
        assert_eq!(word(6.0), "OK");
        assert_eq!(word(3.5), "SLOPPY");
        assert_eq!(word(0.0), "BAD");
    }
}
