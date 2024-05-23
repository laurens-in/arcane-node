use nrf52840_hal::pac;

const PDM_DATA_PORT: bool = false;
const PDM_DATA_PIN: u8 = 0x08;
const PDM_CLOCK_PORT: bool = true;
const PDM_CLOCK_PIN: u8 = 0x09;

pub struct Microphone {
    pdm: pac::PDM,
}

#[allow(dead_code)]
impl Microphone {
    pub fn new(pdm: pac::PDM) -> Self {
        Microphone { pdm }
    }
    pub fn enable(&mut self) {
        // configure the PDM to use the correct pins
        self.pdm.psel.clk.write(|w| {
            w.port().bit(PDM_CLOCK_PORT);
            unsafe {
                w.pin().bits(PDM_CLOCK_PIN);
            }
            w.connect().clear_bit()
        });
        self.pdm.psel.din.write(|w| unsafe {
            w.port().bit(PDM_DATA_PORT);
            w.pin().bits(PDM_DATA_PIN);
            w.connect().clear_bit()
        });
        // enable the PDM
        self.pdm.enable.write(|w| w.enable().set_bit());
        // mono/rising edge
        self.pdm.mode.write(|w| {
            w.operation().set_bit();
            w.edge().clear_bit()
        });
    }
    pub fn enable_interrupt() {}
    pub fn disable_interrupt() {}
    pub fn set_sample_buffer(&self, buf: &[i16]) {
        unsafe {
            self.pdm.sample.ptr.write(|w| w.bits(buf.as_ptr() as u32));
            self.pdm.sample.maxcnt.write(|w| w.bits(buf.len() as u32));
        }
    }
    pub const MIN_GAIN_HALFDB: i8 = -40;
    pub const MAX_GAIN_HALFDB: i8 = 40;
    pub fn set_gain(&self, half_db_gain: i8) -> i8 {
        let mut g = half_db_gain;
        unsafe {
            if half_db_gain < Microphone::MIN_GAIN_HALFDB {
                g = Microphone::MIN_GAIN_HALFDB;
            } else if half_db_gain > Microphone::MIN_GAIN_HALFDB {
                g = Microphone::MAX_GAIN_HALFDB;
            }
            // The gain is offset by the minimum gain to 0, so adjust
            // the signed value here to match what the register wants.
            g -= Microphone::MIN_GAIN_HALFDB;
            self.pdm.gainl.write(|w| w.gainl().bits(g as u8));
            self.pdm.gainr.write(|w| w.gainr().bits(g as u8));
        }
        g
    }

    pub fn start_sampling(&self) {
        self.pdm.tasks_start.write(|w| w.tasks_start().set_bit());
    }

    pub fn sampling_started(&self) -> bool {
        self.pdm.events_started.read().events_started().bit_is_set()
    }

    pub fn clear_sampling_started(&self) {
        self.pdm
            .events_started
            .write(|w| w.events_started().clear_bit());
    }

    pub fn stop_sampling(&self) {
        self.pdm.tasks_stop.write(|w| w.tasks_stop().set_bit());
    }
    pub fn sampling_stopped(&self) -> bool {
        self.pdm.events_stopped.read().events_stopped().bit_is_set()
    }
    pub fn clear_sampling_stopped(&self) {
        self.pdm
            .events_stopped
            .write(|w| w.events_stopped().clear_bit());
    }

    pub fn sampling_ended(&self) -> bool {
        self.pdm.events_end.read().events_end().bit_is_set()
    }

    pub const IRQ_SAMPLING_STARTED: u32 = 0b001;
    pub const IRQ_SAMPLING_STOPPED: u32 = 0b010;
    pub const IRQ_SAMPLING_ENDED: u32 = 0b100;
    pub const IRQ_SAMPLING_ALL: u32 = 0b111;
    pub const IRQ_SAMPLING_NONE: u32 = 0b000;
    pub fn enable_interrupts(&self) {
        unsafe {
            self.pdm
                .inten
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ALL))
        }
    }

    pub fn disable_interrupts(&self) {
        unsafe {
            self.pdm
                .inten
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_NONE))
        }
    }

    pub fn enable_started_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STARTED))
        }
    }
    pub fn disable_started_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STARTED))
        }
    }

    pub fn enable_stopped_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STOPPED))
        }
    }
    pub fn disable_stopped_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_STOPPED))
        }
    }

    pub fn enable_ended_interrupt(&self) {
        unsafe {
            self.pdm
                .intenset
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ENDED))
        }
    }
    pub fn disable_ended_interrupt(&self) {
        unsafe {
            self.pdm
                .intenclr
                .write(|w| w.bits(Microphone::IRQ_SAMPLING_ENDED))
        }
    }
}
