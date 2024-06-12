use defmt::Format;
use serde::Deserialize;

#[derive(Deserialize, Format, Clone)]
pub struct Config {
    pub id: u8,
    pub parameters: Parameters,
}

#[derive(Deserialize, Format, Clone)]
pub struct Parameter<T> {
    pub index: u8,
    pub value: T,
}

#[derive(Deserialize, Format, Clone)]
pub struct Parameters {
    pub gain: Parameter<i8>,
    pub threshold: Parameter<f32>,
    pub length: Parameter<u64>,
    pub throttle: Parameter<u64>,
    pub note: Parameter<u8>,
    pub channel: Parameter<u8>,
}
