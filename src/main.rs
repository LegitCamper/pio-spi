#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::{
    Assembler, InSource, MovDestination, MovOperation, MovSource, OutDestination, SideSet, pio_asm,
};
use embassy_rp::pio::{
    self, Common, Direction, Instance, InterruptHandler, LoadedProgram, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::spi::{self, Error, Phase, Polarity};
use embassy_time::Timer;
use embedded_hal::spi::SpiBus;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Running");
    let p = embassy_rp::init(Default::default());
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let mut config = spi::Config::default();
    config.frequency = 400_000;

    info!("spi freq: {}", config.frequency);
    match config.polarity {
        Polarity::IdleLow => info!("spi pol: idle low"),
        Polarity::IdleHigh => info!("spi pol: idle high"),
    }
    match config.phase {
        Phase::CaptureOnFirstTransition => info!("spi pha: capture first"),
        Phase::CaptureOnSecondTransition => info!("spi pha: capture second"),
    }

    let clk_pin = p.PIN_2; // SPI Clock
    let mosi_pin = p.PIN_3; // SPI MOSI (Master Out)
    let miso_pin = p.PIN_4; // SPI MISO (Master In)
    let cs_pin = p.PIN_5; // Chip Select

    let programs = PioSpiPrograms::new(&mut common, &config);
    let spi = PioSpi::new(
        &mut common,
        sm0,
        clk_pin,
        mosi_pin,
        miso_pin,
        programs,
        config,
    );

    let cs = Output::new(cs_pin, Level::High);
    let spi_dev = unwrap!(ExclusiveDevice::new_no_delay(spi, cs));

    let sdcard = SdCard::new(spi_dev, embassy_time::Delay);
    info!("card type: {}", sdcard.get_card_type());

    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimeSource());

    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

    // Open the root directory (mutably borrows from the volume).
    let mut root_dir = volume0.open_root_dir().unwrap();

    root_dir
        .iterate_dir(|file| info!("File: {}", file.name))
        .unwrap();

    info!("Done!!!");

    loop {}
}

fn polarity_bit(polarity: Polarity, on: bool) -> u8 {
    match polarity {
        Polarity::IdleLow => match on {
            true => 1,
            false => 0,
        },
        Polarity::IdleHigh => match on {
            true => 0,
            false => 1,
        },
    }
}

pub struct PioSpiPrograms<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSpiPrograms<'d, PIO> {
    pub fn new(pio: &mut Common<'d, PIO>, config: &spi::Config) -> Self {
        let mut assembler = Assembler::<32>::new_with_side_set(SideSet::new(false, 1, false));
        let pol = config.polarity;

        match config.phase {
            Phase::CaptureOnFirstTransition => {
                assembler.out_with_delay_and_side_set(
                    OutDestination::PINS,
                    1,
                    1,
                    polarity_bit(pol, false),
                );
                assembler.in_with_delay_and_side_set(InSource::PINS, 1, 1, polarity_bit(pol, true));
            }
            Phase::CaptureOnSecondTransition => {
                assembler.out_with_side_set(OutDestination::X, 1, polarity_bit(pol, false));
                assembler.mov_with_delay_and_side_set(
                    MovDestination::PINS,
                    MovOperation::None,
                    MovSource::X,
                    1,
                    polarity_bit(pol, true),
                );
                assembler.in_with_side_set(InSource::PINS, 1, polarity_bit(pol, false));
            }
        }

        // assert!(config.phase == Phase::CaptureOnFirstTransition);
        // assert!(config.polarity == Polarity::IdleLow);
        // let prg = pio_asm!(
        //     ".side_set 1",
        //     "out pins, 1 side 0 [1]", // ; Stall here on empty (sideset proceeds even if
        //     "in pins, 1  side 1 [1]", // ; instruction stalls, so we stall with SCK low)
        //     options(max_program_size = 32)  // Optional, defaults to 32
        // );

        Self {
            // prg: pio.load_program(&prg.program),
            prg: pio.load_program(&assembler.assemble_program()),
        }
    }
}

pub struct PioSpi<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioSpi<'d, PIO, SM> {
    pub fn new(
        pio: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        clk_pin: impl PioPin,
        mosi_pin: impl PioPin,
        miso_pin: impl PioPin,
        programs: PioSpiPrograms<'d, PIO>,
        config: spi::Config,
    ) -> Self {
        let clk = pio.make_pio_pin(clk_pin);
        let mosi = pio.make_pio_pin(mosi_pin);
        let miso = pio.make_pio_pin(miso_pin);

        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.prg, &[&clk]);
        cfg.set_out_pins(&[&mosi]);
        cfg.set_in_pins(&[&miso]);
        cfg.clock_divider = ((clk_sys_freq() / config.frequency) as u16).into();

        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 8;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_in = shift_cfg;
        cfg.shift_out = shift_cfg;

        sm.set_config(&cfg);
        sm.set_enable(true);
        sm.set_rx_threshold(8);
        sm.set_tx_threshold(8);
        sm.set_pin_dirs(Direction::Out, &[&clk, &mosi]);
        sm.set_pin_dirs(Direction::In, &[&miso]);

        Self { sm }
    }

    fn write_byte(&mut self, byte: u8) {
        info!("Write {:x}", byte);
        self.sm.tx().push((byte as u32) << 24);
    }

    fn read_byte(&mut self) -> u8 {
        let read = self.sm.rx().pull();
        let byte = (read & 0xFF) as u8;
        info!("Read byte: {:02X}", byte);
        byte
    }
}

impl<'d, T: Instance, const SM: usize> embedded_hal::spi::ErrorType for PioSpi<'d, T, SM> {
    type Error = Error;
}

impl<'a, PIO: Instance, const SM: usize> SpiBus<u8> for PioSpi<'a, PIO, SM> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            self.write_byte(0x00);
            while self.sm.rx().empty() {}
            *word = self.read_byte() as u8;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &byte in words {
            while self.sm.tx().full() {}

            self.write_byte(byte);
        }

        self.flush()
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        for (r, &w) in read.iter_mut().zip(write.iter()) {
            while self.sm.tx().full() {}
            self.write_byte(w);

            while self.sm.rx().empty() {}
            *r = self.read_byte();
        }

        self.flush()
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            while self.sm.tx().full() {}
            self.write_byte(*word);

            while self.sm.rx().empty() {}
            *word = self.read_byte();
        }

        self.flush()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.sm.tx().empty() {}
        Ok(())
    }
}

pub struct DummyTimeSource();
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(2022, 1, 1, 0, 0, 0).unwrap()
    }
}
