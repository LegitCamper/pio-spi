#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
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
use embassy_rp::spi::{self, Error, Phase, Polarity, Spi};
use embassy_time::Timer;
use embedded_hal::spi::SpiBus;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rand::{Rng, RngCore};
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

    let clk_pin = p.PIN_2; // SPI Clock
    let mosi_pin = p.PIN_3; // SPI MOSI (Master Out)
    let miso_pin = p.PIN_4; // SPI MISO (Master In)
    let cs_pin = p.PIN_5; // Chip Select
    let mut cs = Output::new(cs_pin, Level::High);

    let programs = PioSpiPrograms::new(&mut common, &config);
    let mut spi = PioSpi::new(
        &mut common,
        sm0,
        clk_pin,
        mosi_pin,
        miso_pin,
        programs,
        config,
    );
    // let spi = Spi::new_blocking(p.SPI0, clk_pin, mosi_pin, miso_pin, config);

    // spi_read_files(spi, cs);

    manual_sd_init(spi, cs).await;

    info!("Done!!!");

    loop {}
}

async fn manual_sd_init<'d, PIO: pio::Instance, const SM: usize>(
    mut spi: PioSpi<'d, PIO, SM>,
    mut cs: Output<'_>,
) {
    info!("Starting manual SD init");

    // 1. Send 80 clocks with CS high
    cs.set_high();
    for _ in 0..10 {
        spi.write_byte(0xFF);
        let _ = spi.read_byte(); // Consume response to keep PIO happy
    }
    Timer::after_millis(10).await;
    info!("Done with 80 clk high");

    // 2. CMD0: GO_IDLE_STATE
    cs.set_low();
    send_cmd(&mut spi, 0, 0x00000000, 0x95);
    let mut response = 0xFF;
    for i in 0..8 {
        response = spi.read_byte();
        if response != 0xFF {
            info!("CMD0 response received at try {}: {:02X}", i, response);
            break;
        }
        Timer::after_millis(1).await;
    }
    cs.set_high();
    spi.write_byte(0xFF);
    let _ = spi.read_byte();
    info!("Sdcard entered idle state");

    if response != 0x01 {
        warn!("CMD0 did not return IDLE (0x01), got: {:02X}", response);
    }

    //3 . CMD8: SEND_IF_COND
    cs.set_low();
    send_cmd(&mut spi, 8, 0x000001AA, 0x87);
    let mut r1 = 0xFF;
    for i in 0..8 {
        r1 = spi.read_byte();
        if r1 != 0xFF {
            info!("CMD8 R1 response at try {}: {:02X}", i, r1);
            break;
        }
        Timer::after_millis(1).await;
    }

    let mut r7 = [0u8; 4];
    for b in r7.iter_mut() {
        *b = spi.read_byte();
    }
    cs.set_high();
    spi.write_byte(0xFF);
    let _ = spi.read_byte();

    info!(
        "CMD8 R1: {:02X}, R7: {:02X} {:02X} {:02X} {:02X}",
        r1, r7[0], r7[1], r7[2], r7[3]
    );
}

fn send_cmd<'d, PIO: pio::Instance, const SM: usize>(
    spi: &mut PioSpi<'d, PIO, SM>,
    cmd: u8,
    arg: u32,
    crc: u8,
) {
    spi.write_byte(0x40 | cmd);
    spi.write_byte((arg >> 24) as u8);
    spi.write_byte((arg >> 16) as u8);
    spi.write_byte((arg >> 8) as u8);
    spi.write_byte(arg as u8);
    spi.write_byte(crc);
}

fn spi_read_files<'d, PIO: pio::Instance, const SM: usize>(
    spi: PioSpi<'d, PIO, SM>,
    cs: Output<'_>,
) {
    let spi_dev = unwrap!(ExclusiveDevice::new_no_delay(spi, cs));

    let sdcard = SdCard::new(spi_dev, embassy_time::Delay);
    info!("SPI Pio driver finished init");

    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
    sdcard.spi(|dev| dev.bus_mut().set_config(&config));

    let mut volume_mgr = VolumeManager::new(sdcard, DummyTimeSource());
    info!("Got volume manager");

    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0)).unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));
    info!("Got volume 0");

    let mut root_dir = volume0.open_root_dir().unwrap();

    root_dir
        .iterate_dir(|file| info!("File: {:?}", str::from_utf8(file.name.base_name()).unwrap()))
        .unwrap();
}

fn spi_loopback_test<'d, PIO: pio::Instance, const SM: usize>(spi: &mut PioSpi<'d, PIO, SM>) {
    let mut rng = RoscRng;

    for i in 0..100 {
        let byte = rng.r#gen::<u8>();
        spi.write_byte(byte);
        let received = spi.read_byte();
        if received != byte {
            warn!(
                "Mismatch at iteration {}: sent {:02X}, got {:02X}",
                i, byte, received
            );
            break;
        } else {
            info!("OK @ {}: {:02X}", i, byte);
        }
    }
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

        Self {
            prg: pio.load_program(&assembler.assemble_program()),
        }
    }
}

pub struct PioSpi<'d, PIO: Instance, const SM: usize> {
    config: pio::Config<'d, PIO>,
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
        sm.set_rx_threshold(8);
        sm.set_tx_threshold(8);
        sm.set_pin_dirs(Direction::Out, &[&clk, &mosi]);
        sm.set_pin_dirs(Direction::In, &[&miso]);
        sm.set_enable(true);

        Self { sm, config: cfg }
    }

    pub fn set_config(&mut self, config: &spi::Config) {
        self.config.clock_divider = ((clk_sys_freq() / config.frequency) as u16).into();
        self.sm.restart();
    }

    fn write_byte(&mut self, byte: u8) {
        while self.sm.tx().full() {}
        self.sm.tx().push((byte as u32) << 24);
        // info!("Write {:x}", byte);
    }

    fn read_byte(&mut self) -> u8 {
        while self.sm.rx().empty() {}
        let read = self.sm.rx().pull();
        let byte = (read & 0xFF) as u8;
        // info!("Read byte: {:02X}", byte);
        byte
    }
}

impl<'d, T: Instance, const SM: usize> embedded_hal::spi::ErrorType for PioSpi<'d, T, SM> {
    type Error = Error;
}

impl<'a, PIO: Instance, const SM: usize> SpiBus<u8> for PioSpi<'a, PIO, SM> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            self.write_byte(0xFF);
            *word = self.read_byte() as u8;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &byte in words {
            self.write_byte(byte);
        }

        self.flush()
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        for (r, &w) in read.iter_mut().zip(write.iter()) {
            self.write_byte(w);
            *r = self.read_byte();
        }

        self.flush()
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            self.write_byte(*word);
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
