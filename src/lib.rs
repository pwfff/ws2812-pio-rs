#![no_std]
//! WS2812 PIO Driver for the RP2040
//!
//! This driver implements driving a WS2812 RGB LED strip from
//! a PIO device of the RP2040 chip.
//!
//! You should reach to [Ws2812] if you run the main loop
//! of your controller yourself and you want [Ws2812] to take
//! a hold of your timer.
//!
//! In case you use `cortex-m-rtic` and can't afford this crate
//! to wait blocking for you, you should try [Ws2812Direct].
//! Bear in mind that you will have to take care of timing requirements
//! yourself then.

use cortex_m::{self};
use fugit::{ExtU32, HertzU32, HertzU64};
// use morton_encoding::morton_encode;
use defmt::info;
use defmt_rtt as _;
use nb::block;
use rp2040_hal::{
    dma::{
        double_buffer::{Config, ReadNext, Transfer},
        SingleChannel,
    },
    gpio::{Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{InstalledProgram, PIOExt, StateMachineIndex, Tx, UninitStateMachine, PIO},
};
use smart_leds_trait::RGB8;

pub struct LEDs<const SIZE: usize, K: Into<RGB8>> {
    pub channel0: [K; SIZE],
    pub channel1: [K; SIZE],
}

// TODO: buffer size only needs to be like... * 3/2?
#[macro_export]
macro_rules! buf {
    ($size:expr) => {
        singleton!(: [u32; $size * 2] = [0; $size * 2]).unwrap()
    };
}

pub type LEDBuf<const SIZE: usize> = [u32; SIZE];

// https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
impl<const S: usize, K: Into<RGB8> + Copy> LEDs<S, K> {
    pub fn fill<const BS: usize>(&self, buf: &mut LEDBuf<BS>) {
        let mut buf_i = 0;
        for i in 0..S {
            // TODO: faster to encode_u8 thrice,
            //  or encode_u32 once and throw away last byte?
            let c1: RGB8 = self.channel0[i].into();
            let c2: RGB8 = self.channel1[i].into();
            let g = morton_encode_u8(c1.g, c2.g);
            let r = morton_encode_u8(c1.r, c2.r);
            let b = morton_encode_u8(c1.b, c2.b);

            if i % 2 == 0 {
                buf[buf_i] = u32::from(g) << 16 | u32::from(r);
                buf[buf_i + 1] = u32::from(b) << 16;
                buf_i += 1
            } else {
                buf[buf_i] |= u32::from(g);
                buf[buf_i + 1] = u32::from(r) << 16 | u32::from(b);
                buf_i += 2
            }
        }
    }
}

pub fn morton_encode_u8(x: u8, y: u8) -> u16 {
    // insert a 0 bit between each of the 32 bits of x and y
    // (truncated) example:
    // 0b1111 -> double size, 0b00001111 -> bloat 0b01010101
    let x = bloat_u8(x);
    let y = bloat_u8(y);

    // bump x 1 bit to the left, and smush it together with y
    // (truncated) example:
    // 0b01010101 ----------------> 0b11111111
    // 0b01010101 -> 0b10101010 -'
    (y << 1) | x
}

fn bloat_u8(x: u8) -> u16 {
    // x = ---- ---- ---- ---- ---- ---- ---- ---- fedc ba98 7654 3210 fedc ba98 7654 3210
    const masks: [(u8, u16); 3] = [
        (4, 0x0f0f), //  ---- fedc ---- ba98
        (2, 0x3333), //  --fe --dc --ba --98
        (1, 0x5555), //  -f-e -d-c -b-a -9-8
    ];
    let mut x = x as u16;

    x = (x ^ (x << masks[0].0)) & masks[0].1;
    x = (x ^ (x << masks[1].0)) & masks[1].1;
    x = (x ^ (x << masks[2].0)) & masks[2].1;

    x
}

pub fn morton_encode_u32(x: u32, y: u32) -> u64 {
    // insert a 0 bit between each of the 32 bits of x and y
    // (truncated) example:
    // 0b1111 -> double size, 0b00001111 -> bloat 0b01010101
    let x = bloat_u32(x);
    let y = bloat_u32(y);

    // bump x 1 bit to the left, and smush it together with y
    // (truncated) example:
    // 0b01010101 ----------------> 0b11111111
    // 0b01010101 -> 0b10101010 -'
    (y << 1) | x
}

fn bloat_u32(x: u32) -> u64 {
    // x = ---- ---- ---- ---- ---- ---- ---- ---- fedc ba98 7654 3210 fedc ba98 7654 3210
    const masks: [(u8, u64); 5] = [
        (16, 0x0000ffff0000ffff), // ---- ---- ---- ---- fedc ba98 7654 3210 ---- ---- ---- ---- fedc ba98 7654 3210
        (8, 0x00ff00ff00ff00ff), //  ---- ---- fedc ba98 ---- ---- 7654 3210 ---- ---- fedc ba98 ---- ---- 7654 3210
        (4, 0x0f0f0f0f0f0f0f0f), //  ---- fedc ---- ba98 ---- 7654 ---- 3210 ---- fedc ---- ba98 ---- 7654 ---- 3210
        (2, 0x3333333333333333), //  --fe --dc --ba --98 --76 --54 --32 --10 --fe --dc --ba --98 --76 --54 --32 --10
        (1, 0x5555555555555555), //  -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0 -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
    ];
    let mut x = x as u64;

    x = (x ^ (x << masks[0].0)) & masks[0].1;
    x = (x ^ (x << masks[1].0)) & masks[1].1;
    x = (x ^ (x << masks[2].0)) & masks[2].1;
    x = (x ^ (x << masks[3].0)) & masks[3].1;
    x = (x ^ (x << masks[4].0)) & masks[4].1;

    x
}

pub fn init<P: PIOExt + FunctionConfig>(pio: &mut PIO<P>) -> InstalledProgram<P> {
    // better (?) implementation from adafruit? https://learn.adafruit.com/intro-to-rp2040-pio-with-circuitpython/using-pio-to-drive-a-neopixel
    /*
    .program ws2812
    .side_set 1
    .wrap_target
    bitloop:
       out x 1        side 0 [6]; Drive low. Side-set still takes place before instruction stalls.
       jmp !x do_zero side 1 [3]; Branch on the bit we shifted out previous delay. Drive high.
     do_one:
       jmp  bitloop   side 1 [4]; Continue driving high, for a one (long pulse)
     do_zero:
       nop            side 0 [4]; Or drive low, for a zero (short pulse)
    .wrap
    */

    // cycle times here are: _______----_____
    // 7 (out + 6), - low    _______---------
    // 4 (jmp + 3), - high
    // 5 (jmp/nop + 4) - high/low
    // = 16

    // left side is branch target
    // X--
    // |\
    // | - was 00, write 11 for 4, 00 for 5, wrap
    // X-1
    // |\- was 01, write 11 for 4, 01 for 5, wrap
    // X-2
    // |\- was 10, write 11 for 4, 10 for 5, wrap
    // X-3
    // \-  was 11, write 11 for 9, wrap

    // modified to drive two pins from one bitstream.
    // note that the bitstream must be pre-munged.
    /*
    .program ws2812
    .side_set 2;                      we'll be side-setting two bits at a time to drive the two pins
    .wrap_target
    bitloop:
        out x 2        side 0b00 [0]; drive pins low, get pin bits. side-set still takes place before instruction stalls. cycle: 1
        jmp x-- do_01  side 0b00 [0]; branch-then-decrement when X > 0. delay always happens.                                     2
        nop            side 0b00 [4]; finish low period.                                                                           3---7
        nop            side 0b11 [3]; single high pulse for both.                                                                       8-11      _______----_____
        jmp bitloop    side 0b00 [4]; low pulse for both 0s, back to beginning.                                                             12-16 _______----_____
    do_01:
        jmp x-- do_10  side 0b00 [0]; branch-then-decrement when X > 0.                                                            3
        nop            side 0b00 [3]; finish low period.                                                                            4--7
        nop            side 0b11 [3]; single high pulse for both.                                                                       8-11      _______----_____
        jmp bitloop    side 0b01 [4]; low pulse for pin0, high pulse for pin1, back to beginning.                                           12-16 _______---------
    do_10:
        jmp x-- do_11  side 0b00 [0]; branch-then-decrement when X > 0.                                                             4
        nop            side 0b00 [2]; finish low period.                                                                             5-7
        nop            side 0b11 [3]; single high pulse for both.                                                                       8-11      _______---------
        jmp bitloop    side 0b10 [4]; low pulse for pin0, high pulse for pin1, back to beginning.                                           12-16 _______----_____
    do_11:
        nop            side 0b00 [2]; finish low period.                                                                             5-7           _______---------
        nop            side 0b11 [7]; single high pulse for both, all the way through.                                                  8------16  _______---------
        nop            side 0b11 [0]; how embarassing, we can't delay for 8...
    .wrap
    */

    // prepare the PIO program
    let side_set = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(side_set);

    let mut wrap_target = a.label(); // also bitloop
    let mut wrap_source = a.label();
    let mut do_01 = a.label();
    let mut do_10 = a.label();
    let mut do_11 = a.label();
    a.bind(&mut wrap_target);
    // drive pins low, get our two bits. side-set still takes place even if stalled.
    a.out_with_delay_and_side_set(pio::OutDestination::X, 2, 0, 0b00);
    // check if x > 0, jumping if so. always decrements. still no delay, side-set still 0.
    a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut do_01, 0, 0b00);
    // if x was 0, both our bits are low, so finish our initial low period, do a short high then low.
    a.nop_with_delay_and_side_set(4, 0b00);
    a.nop_with_delay_and_side_set(3, 0b11);
    a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, 4, 0b00);

    // if x was > 0, we jumped here.
    a.bind(&mut do_01);
    // check if x > 0 again, jumping if so. always decrements. still no delay, side-set still 0.
    a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut do_10, 0, 0b00);
    // if x was 0 here, we got 0b01, so finish initial low, do a short high, then low for pin 1 and high for pin 0.
    a.nop_with_delay_and_side_set(3, 0b00);
    a.nop_with_delay_and_side_set(3, 0b11);
    a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, 4, 0b01);

    // if x was still > 0, we jumped here.
    a.bind(&mut do_10);
    // check if x > 0 again, jumping if so. always decrements. still no delay, side-set still 0. (could be just !x i guess)
    a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut do_11, 0, 0b00);
    // if x was 0 here, we got 0b10, so finish initial low, do a short high, then high for pin 1 and low for pin 0.
    a.nop_with_delay_and_side_set(2, 0b00);
    a.nop_with_delay_and_side_set(3, 0b11);
    a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, 4, 0b10);

    // finally, x must have been 0b11
    a.bind(&mut do_11);
    // finish initial low, do a short high, then high for pin 1 and low for pin 0.
    a.nop_with_delay_and_side_set(2, 0b00);
    a.nop_with_delay_and_side_set(7, 0b11);
    a.nop_with_delay_and_side_set(0, 0b11);
    a.bind(&mut wrap_source);
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Install the program into PIO instruction memory.
    pio.install(&program).unwrap()
}

/// This is the WS2812 PIO Driver.
///
/// For blocking applications is recommended to use
/// the [Ws2812] struct instead of this raw driver.
///
/// If you use this driver directly, you will need to
/// take care of the timing expectations of the [Ws2812Direct::write]
/// method.
///
/// Typical usage example:
///```ignore
/// use rp2040_hal::clocks::init_clocks_and_plls;
/// let clocks = init_clocks_and_plls(...);
/// let pins = rp2040_hal::gpio::pin::bank0::Pins::new(...);
///
/// let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
/// let mut ws = Ws2812Direct::new(
///     pins.gpio4.into_mode(),
///     &mut pio,
///     sm0,
///     clocks.peripheral_clock.freq(),
/// );
///
/// // Then you will make sure yourself to not write too frequently:
/// loop {
///     use smart_leds::{SmartLedsWrite, RGB8};
///     let color : RGB8 = (255, 0, 255).into();
///
///     ws.write([color].iter().copied()).unwrap();
///     delay_for_at_least_60_microseconds();
/// };
///```
pub struct Ws2812Direct<P, SM, I, J, CH1, CH2, const BS: usize, T>
where
    I: PinId,
    J: PinId,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
    T: embedded_hal::timer::CountDown,
    T::Time: From<fugit::MicrosDurationU64>,
{
    tx_transfer:
        Transfer<CH1, CH2, &'static mut LEDBuf<BS>, Tx<(P, SM)>, ReadNext<&'static mut LEDBuf<BS>>>,
    _pin0: Pin<I, Function<P>>,
    _pin1: Pin<J, Function<P>>,
    timer: T,
    pub led_us: fugit::MicrosDurationU64,
}

impl<P, SM, I, J, CH1, CH2, const BS: usize, T> Ws2812Direct<P, SM, I, J, CH1, CH2, BS, T>
where
    I: PinId,
    J: PinId,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
    T: embedded_hal::timer::CountDown,
    T::Time: From<fugit::MicrosDurationU64>,
{
    /// Creates a new instance of this driver.
    pub fn new(
        installed: InstalledProgram<P>,
        pin0: Pin<I, Function<P>>,
        pin1: Pin<J, Function<P>>,
        dma: (CH1, CH2),
        buf0: &'static mut LEDBuf<BS>,
        buf1: &'static mut LEDBuf<BS>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: fugit::HertzU32,
        mut timer: T,
    ) -> Self {
        assert!(
            I::DYN.num == J::DYN.num - 1,
            "Provided pins must be next to each other"
        );

        const CYCLES_PER_BIT: u32 = 16;
        const FREQ: HertzU32 = HertzU32::kHz(800);

        // Configure the PIO state machine.
        let bit_freq = FREQ * CYCLES_PER_BIT;
        let mut int = clock_freq / bit_freq;
        let rem = clock_freq - (int * bit_freq);
        let frac = (rem * 256) / bit_freq;
        assert!(
            (1..=65536).contains(&int) && (int != 65536 || frac == 0),
            "(System Clock / {}) must be within [1.0, 65536.0].",
            bit_freq.to_kHz()
        );

        // 65536.0 is represented as 0 in the pio's clock divider
        if int == 65536 {
            int = 0;
        }
        // Using lossy conversion because range have been checked
        let int: u16 = int as u16;
        let frac: u8 = frac as u8;

        let (mut sm, _, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            // only use TX FIFO
            .buffers(rp2040_hal::pio::Buffers::OnlyTx)
            // Pin configuration
            .side_set_pin_base(I::DYN.num)
            // OSR config
            .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
            .autopull(true)
            .clock_divisor_fixed_point(int, frac)
            .build(sm);

        // Prepare pin's direction.
        sm.set_pindirs([
            (I::DYN.num, rp2040_hal::pio::PinDir::Output),
            (J::DYN.num, rp2040_hal::pio::PinDir::Output),
        ]);

        // number of us to light one LED
        let foo: HertzU64 = FREQ.into();
        let led_us = (foo / 24 / ((buf0.len() / 2).max(buf0.len() / 2) as u32)).into_duration()
            + 150_u32.micros();
        info!("poop {}", led_us.to_micros());

        // TODO: fix this buffer size led count nonsense...
        timer.start(led_us);
        let tx_transfer = Config::new(dma, buf0, tx).start().read_next(buf1);

        sm.start();

        Self {
            tx_transfer,
            _pin0: pin0,
            _pin1: pin1,
            timer,
            led_us,
        }
    }

    pub fn write<const S: usize, K>(mut self, leds: &LEDs<S, K>) -> Self
    where
        K: Into<RGB8> + Copy,
    {
        // TODO: ugh just make this an iterator and if it's too big, whatever, i guess?
        // TODO: double is not right though? it's like... (*2)/3?
        assert!(BS == S * 2, "buffer sizes must be LED count * 2");

        let (tx_buf, next_tx_transfer) = self.tx_transfer.wait();

        // make sure last value has latched
        block!(self.timer.wait()).unwrap();

        leds.fill(tx_buf);

        self.tx_transfer = next_tx_transfer.read_next(tx_buf);

        self.timer.start(self.led_us);

        self
    }

    // pub fn pump(mut self) -> Self {
    //     let (tx_buf, next_tx_transfer) = self.tx_transfer.wait();
    //     self.tx_transfer = next_tx_transfer.read_next(tx_buf);
    //     self
    // }
}

pub trait DelayUs {
    /// Pauses execution for `us` microseconds
    fn delay_us(&mut self, us: u32);
}

/*
impl<P, SM, I, J, CH1, CH2, const SIZE: usize> SmartLedsWrite
    for Ws2812Direct<P, SM, I, J, CH1, CH2, SIZE>
where
    I: PinId,
    J: PinId,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
{
    type Color = smart_leds_trait::RGB8;
    type Error = ();
    /// If you call this function, be advised that you will have to wait
    /// at least 60 microseconds between calls of this function!
    /// That means, either you get hold on a timer and the timing
    /// requirements right your self, or rather use [Ws2812].
    ///
    /// Please bear in mind, that it still blocks when writing into the
    /// PIO FIFO until all data has been transmitted to the LED chain.
    fn write<T, K>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = K>,
        K: Into<Self::Color>,
    {
        let (tx_buf, next_tx_transfer) = self.tx.wait();

        for (i, item) in iterator.enumerate() {
            if i > tx_buf.len() {
                break;
            }

            let color: Self::Color = item.into();
            let word =
                (u32::from(color.g) << 24) | (u32::from(color.r) << 16) | (u32::from(color.b) << 8);
            tx_buf[i] = word;
        }

        self.tx = next_tx_transfer.read_next(tx_buf);

        Ok(())
    }
}

/// Instance of a WS2812 LED chain.
///
/// Use the [Ws2812::write] method to update the WS2812 LED chain.
///
/// Typical usage example:
///```ignore
/// use rp2040_hal::clocks::init_clocks_and_plls;
/// let clocks = init_clocks_and_plls(...);
/// let pins = rp2040_hal::gpio::pin::bank0::Pins::new(...);
///
/// let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
///
/// let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
/// let mut ws = Ws2812::new(
///     pins.gpio4.into_mode(),
///     &mut pio,
///     sm0,
///     clocks.peripheral_clock.freq(),
///     timer.count_down(),
/// );
///
/// loop {
///     use smart_leds::{SmartLedsWrite, RGB8};
///     let color : RGB8 = (255, 0, 255).into();
///
///     ws.write([color].iter().copied()).unwrap();
///
///     // Do other stuff here...
/// };
///```
pub struct Ws2812<P, SM, C, I, J, CH1, CH2, const SIZE: usize>
where
    I: PinId,
    J: PinId,
    C: CountDown,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
{
    driver: Ws2812Direct<P, SM, I, J, CH1, CH2, SIZE>,
    cd: C,
}

impl<P, SM, C, I, J, CH1, CH2, const SIZE: usize> Ws2812<P, SM, C, I, J, CH1, CH2, SIZE>
where
    I: PinId,
    J: PinId,
    C: CountDown,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    pub fn new(
        pin0: Pin<I, Function<P>>,
        pin1: Pin<J, Function<P>>,
        pio: &mut PIO<P>,
        dma: (CH1, CH2),
        bufs: (&'static mut [u32; SIZE], &'static mut [u32; SIZE]),
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: fugit::HertzU32,
        cd: C,
    ) -> Ws2812<P, SM, C, I, J, CH1, CH2, SIZE> {
        let driver = Ws2812Direct::new(pin0, pin1, pio, dma, bufs, sm, clock_freq);

        Self { driver, cd }
    }
}

impl<'timer, P, SM, I, J, CH1, CH2, const SIZE: usize> SmartLedsWrite
    for Ws2812<P, SM, rp2040_hal::timer::CountDown<'timer>, I, J, CH1, CH2, SIZE>
where
    I: PinId,
    J: PinId,
    P: PIOExt + FunctionConfig,
    CH1: SingleChannel,
    CH2: SingleChannel,
    Function<P>: ValidPinMode<I>,
    Function<P>: ValidPinMode<J>,
    SM: StateMachineIndex,
{
    type Color = smart_leds_trait::RGB8;
    type Error = ();
    fn write<T, K>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = K>,
        K: Into<Self::Color>,
    {
        // self.driver.tx.clear_stalled_flag();
        // while !self.driver.tx.is_empty() && !self.driver.tx.has_stalled() {}

        // self.cd.start(60u32.micros());
        // let _ = nb::block!(self.cd.wait());

        self.driver.write(iterator)
    }
}

*/
