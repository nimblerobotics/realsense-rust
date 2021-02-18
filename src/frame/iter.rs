//! Generic iterator types for implementing frame iterators.

use super::pixel::PixelKind;
use super::prelude::{VideoFrameEx, VideoFrameUnsafeEx};

pub struct ImageIter<'a, F>
where
    F: VideoFrameEx<'a> + VideoFrameUnsafeEx<'a>,
{
    pub(crate) frame: &'a F,
    pub(crate) column: usize,
    pub(crate) row: usize,
}

impl<'a, F> Iterator for ImageIter<'a, F>
where
    F: VideoFrameEx<'a> + VideoFrameUnsafeEx<'a>,
{
    type Item = PixelKind<'a>;

    /// Provides a row-major iterator over an entire Image.
    fn next(&mut self) -> Option<Self::Item> {
        if self.column >= self.frame.width() || self.row >= self.frame.height() {
            return None;
        }

        let next = self.frame.get_unchecked(self.column, self.row);

        self.column += 1;

        if self.column >= self.frame.width() {
            self.column = 0;
            self.row += 1;
        }
        Some(next)
    }
}
