use defmt::Format;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Format)]
pub struct Config {
    id: u8,
    parameters: Parameters,
}

#[derive(Serialize, Deserialize, Format)]
pub struct Parameter {
    index: u8,
    value: u32,
}

#[derive(Serialize, Deserialize, Format)]
pub struct Parameters {
    gain: Parameter,
    threshold: Parameter,
}
