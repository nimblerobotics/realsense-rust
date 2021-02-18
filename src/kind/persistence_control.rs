//! The enumeration of persistence controls.

use num_derive::{FromPrimitive, ToPrimitive};

#[repr(usize)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PersistenceControl {
    Disabled = 0,
    Valid8OutOf8 = 1,
    Valid2OutOf3 = 2,
    Valid2OutOf4 = 3,
    Valid2OutOf8 = 4,
    Valid1OutOf2 = 5,
    Valid1OutOf5 = 6,
    Valid1OutOf8 = 7,
    Indefinitely = 8,
}
