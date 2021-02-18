//! Common traits provided by the `kind` module.

use super::extension::Rs2Extension;

/// A trait for describing types that characterized by a single [`Rs2Extension`].
pub trait Extension {
    /// Identifies the corresponding [`Rs2Extension`] for the type implementing this trait.
    fn extension() -> Rs2Extension;
}
