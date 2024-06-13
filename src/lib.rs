#![no_main]
#![no_std]

use arcane::{ArcaneCode, ArcaneId};
use config::Config;
use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _;
use embedded_hal::can::{Frame, Id::Standard};
use mic::Microphone; // global logger

use mcp2515::{frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use panic_probe as _;

use nrf52840_hal::{
    self as _,
    gpio::{Input, Output, Pin, PullUp, PushPull},
    gpiote::Gpiote,
    pac::{CLOCK, PDM, SPIM0, SYST},
    spim, Clocks, Delay, Spim,
};

pub mod arcane;
pub mod config;
pub mod mic;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub fn initialize_can(
    cs: Pin<Output<PushPull>>,
    int: &Pin<Input<PullUp>>,
    pins: spim::Pins,
    spim: SPIM0,
    gpiote: &Gpiote,
    syst: SYST,
) -> MCP2515<Spim<SPIM0>, Pin<Output<PushPull>>> {
    let spi = Spim::new(spim, pins, spim::Frequency::M1, spim::MODE_0, 0);

    // Initialize CAN
    let mut delay = Delay::new(syst);
    let mut can = MCP2515::new(spi, cs);

    can.init(
        &mut delay,
        mcp2515::Settings {
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps1000,
            mcp_speed: McpSpeed::MHz16,
            clkout_en: false,
        },
    )
    .unwrap();

    gpiote
        .channel0()
        .input_pin(int)
        .hi_to_lo()
        .enable_interrupt();

    can
}

pub const SAMPLECOUNT: u16 = 128;
pub fn initialize_mic(
    pdm: PDM,
    clock: CLOCK,
    config: &Config,
) -> (Microphone, [[i16; SAMPLECOUNT as usize]; 2], usize, usize) {
    // Initialize MIC
    let mut mic = Microphone::new(pdm);

    // We need the high-frequency oscillator enabled for the microphone
    let c = Clocks::new(clock);
    c.enable_ext_hfosc();

    let pdm_buffers: [[i16; SAMPLECOUNT as usize]; 2] = [[0; SAMPLECOUNT as usize]; 2];

    let buf_to_display: usize = 0;
    let buf_to_capture: usize = 1;

    mic.enable();
    mic.enable_interrupts();
    mic.set_gain(config.parameters.gain.value);

    mic.set_sample_buffer(&pdm_buffers[buf_to_capture]);
    mic.start_sampling();

    (mic, pdm_buffers, buf_to_display, buf_to_capture)
}

pub fn process_cfg(frame: CanFrame, config: &mut Config) {
    match frame.id() {
        Standard(id)
            if id
                == ArcaneId::new(ArcaneCode::CFGW, config.id)
                    .unwrap()
                    .as_standard_id() =>
        {
            defmt::trace!("CFGW message for this node: {:?}", frame);
            // first byte contains parameter index
            match frame.data()[0] {
                index if index == config.parameters.gain.index => {
                    defmt::trace!("Gain parameter index matched: {:?}", index);
                    config.parameters.gain.value = i8::from_be_bytes([frame.data()[1]]);
                }
                index if index == config.parameters.note.index => {
                    defmt::trace!("Note parameter index matched: {:?}", index);
                    config.parameters.note.value = frame.data()[1];
                }
                index if index == config.parameters.channel.index => {
                    defmt::trace!("Channel parameter index matched: {:?}", index);
                    defmt::trace!("Value: {:?}", frame.data()[1]);
                    config.parameters.channel.value = frame.data()[1];
                }
                _ => defmt::trace!("Parameter index not implemented"),
            };
        }
        _ => defmt::trace!("Message for other node: {:?}", frame),
    }
}

pub fn mean(samples: &[i16]) -> f32 {
    let mut avg: f32 = 0.0;
    for s in samples {
        avg += (*s).saturating_abs() as f32;
    }
    avg / (samples.len() as f32)
}
