//! Marker traits and types for [ProcessingBlock](crate::processing_block::ProcessingBlock).

use crate::{common::*, kind::Rs2Extension};

pub trait ProcessingBlockKind {}
pub trait ExtendableProcessingBlockKind
where
    Self: ProcessingBlockKind,
{
    const EXTENSION: Rs2Extension;
}

#[derive(Debug)]
pub struct Any;
impl ProcessingBlockKind for Any {}

#[derive(Debug)]
pub struct DecimationFilterKind;
impl ProcessingBlockKind for DecimationFilterKind {}
impl ExtendableProcessingBlockKind for DecimationFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::DecimationFilter;
}

#[derive(Debug)]
pub struct ThresholdFilterKind;
impl ProcessingBlockKind for ThresholdFilterKind {}
impl ExtendableProcessingBlockKind for ThresholdFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::ThresholdFilter;
}

#[derive(Debug)]
pub struct DisparityFilterKind;
impl ProcessingBlockKind for DisparityFilterKind {}
impl ExtendableProcessingBlockKind for DisparityFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::DisparityFilter;
}

#[derive(Debug)]
pub struct SpatialFilterKind;
impl ProcessingBlockKind for SpatialFilterKind {}
impl ExtendableProcessingBlockKind for SpatialFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::SpatialFilter;
}

#[derive(Debug)]
pub struct TemporalFilterKind;
impl ProcessingBlockKind for TemporalFilterKind {}
impl ExtendableProcessingBlockKind for TemporalFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::TemporalFilter;
}

#[derive(Debug)]
pub struct HoleFillingFilterKind;
impl ProcessingBlockKind for HoleFillingFilterKind {}
impl ExtendableProcessingBlockKind for HoleFillingFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::HoleFillingFilter;
}

#[derive(Debug)]
pub struct ZeroOrderFilterKind;
impl ProcessingBlockKind for ZeroOrderFilterKind {}
impl ExtendableProcessingBlockKind for ZeroOrderFilterKind {
    const EXTENSION: Rs2Extension = Rs2Extension::ZeroOrderFilter;
}

#[derive(Debug)]
pub struct PointCloudKind;
impl ProcessingBlockKind for PointCloudKind {}

#[derive(Debug)]
pub struct YuyDecoderKind;
impl ProcessingBlockKind for YuyDecoderKind {}

#[derive(Debug)]
pub struct UnitsTransformKind;
impl ProcessingBlockKind for UnitsTransformKind {}

#[derive(Debug)]
pub struct SyncerKind;
impl ProcessingBlockKind for SyncerKind {}

#[derive(Debug)]
pub struct AlignKind;
impl ProcessingBlockKind for AlignKind {}

#[derive(Debug)]
pub struct ColorizerKind;
impl ProcessingBlockKind for ColorizerKind {}

#[derive(Debug)]
pub struct HuffmanDepthDecompressKind;
impl ProcessingBlockKind for HuffmanDepthDecompressKind {}

#[derive(Debug)]
pub struct RatesPrinterKind;
impl ProcessingBlockKind for RatesPrinterKind {}
