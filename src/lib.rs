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

extern crate alloc;
use alloc::{boxed::Box, vec::Vec};
use cortex_m::{self, singleton};
use embedded_dma::ReadBuffer;
use fugit::{ExtU32, HertzU32, HertzU64, MicrosDurationU64};
// use morton_encoding::morton_encode;
use defmt::info;
use defmt_rtt as _;
use nb::block;
use rp2040_hal::{
    dma::{
        single_buffer::{Config, Transfer},
        Channel, ChannelIndex, Channels, SingleChannel, CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7,
    },
    gpio::{bank0::BankPinId, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pac::{PIO0, PIO1, RESETS},
    pio::{
        InstalledProgram, PIOBuilder, PIOExt, StateMachineIndex, Tx, UninitStateMachine, PIO, SM0,
        SM1, SM2, SM3,
    },
};
use rp_pico::Pins;
use smart_leds_trait::{RGB, RGB8};

pub struct LEDs<const SIZE: usize, K: Into<RGB8>> {
    pub channel0: [K; SIZE],
    pub channel1: [K; SIZE],
}

impl<const SIZE: usize, K: Into<RGB8> + Default + Copy> LEDs<SIZE, K> {
    pub fn new() -> Self {
        assert!(SIZE < MAX_LEDS, "need a bigger buffer for this");
        Self {
            channel0: [<K>::default(); SIZE],
            channel1: [<K>::default(); SIZE],
        }
    }
}

// TODO: buffer size only needs to be like... * 3/2?
#[macro_export]
macro_rules! leds {
    ($ty:ty, $size:expr) => {{
        LEDs::<$size, $ty>::new()
    }};
}

// https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
impl<const S: usize, K: Into<RGB8> + Copy> LEDs<S, K> {
    pub fn fill(&self, buf: &mut [u32; MAX_BUFFER_WORDS]) {
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

// 1365 LEDs fits in to 1024 bytes
const MAX_LEDS: usize = 10;
pub const MAX_BUFFER_SIZE: u16 = 256;
pub const MAX_BUFFER_WORDS: usize = MAX_BUFFER_SIZE as usize / 4;

pub trait DMATrait {
    const COUNT: usize;

    fn count(&self) -> usize {
        Self::COUNT
    }

    fn bytes(&self) -> u16 {
        (((Self::COUNT * 24) + 31) / 32) as u16
    }

    fn words(&self) -> usize {
        1 << self.ring_size() as usize
    }

    fn ring_size(&self) -> u8 {
        16 - self.bytes().leading_zeros() as u8
    }
}

impl<const COUNT: usize, K: Into<RGB8>> DMATrait for LEDs<COUNT, K> {
    const COUNT: usize = COUNT;
}

// pub const fn trait_size<P: DMATrait>() -> usize {
//     buf_size(P::COUNT)
// }

// pub const fn buf_size(count: usize) -> usize {
//     // bytes we need for this many LEDs, using 24 bits per LED
//     let bytes: u16 = (((count * 24) + 31) / 32) as u16;
//     assert!(bytes <= 32768);
//     // number of u32 words we need for that many bytes
//     // ((ch0_ring_bytes + 3) / 4) as usize

//     // but we want nearest power of two that will fit
//     // (to get ring buffer functionality)

//     // u16, so assuming we need 1234 bytes:
//     // 0b0000 0100 1101 0010 - ch0_bytes
//     // 0b0000 1000 0000 0000 - nearest power of two
//     // ch0_bytes.leading_zeros()      = 5
//     // 16 - ch0_bytes.leading_zeros() = 11
//     // ring size will be 11
//     1 << ring_size(bytes as usize) as usize
// }

pub const fn buf_bytes(count: usize) -> u16 {
    (((count * 24) + 31) / 32) as u16
}

pub const fn buf_words(count: usize) -> usize {
    1 << ring_size(buf_bytes(count) as u16) as usize
}

pub const fn ring_size(bytes: u16) -> u8 {
    16 - bytes.leading_zeros() as u8
}

// pub const fn ring_size(bytes: usize) -> u8 {
//     // u16, so assuming we need 1234 bytes:
//     // 0b0000 0100 1101 0010 - 1234 bytes
//     // 0b0000 1000 0000 0000 - nearest power of two
//     // bytes.leading_zeros()      = 5
//     // 16 - bytes.leading_zeros() = 11
//     // ring size will be 11
//     16 - bytes.leading_zeros() as u8
// }

#[macro_export]
macro_rules! dma_buf {
    () => {
        singleton!(: [u32; $crate::MAX_BUFFER_WORDS] = [0; $crate::MAX_BUFFER_WORDS]).unwrap()
    };
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
pub struct Ws2812Direct<T: embedded_hal::timer::CountDown> {
    timer: T,
    led_us: MicrosDurationU64,
    t0: Transfer<
        Channel<CH0>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO0, SM0)>,
    >,
    t1: Transfer<
        Channel<CH1>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO0, SM1)>,
    >,
    t2: Transfer<
        Channel<CH2>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO0, SM2)>,
    >,
    t3: Transfer<
        Channel<CH3>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO0, SM3)>,
    >,
    t4: Transfer<
        Channel<CH4>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO1, SM0)>,
    >,
    t5: Transfer<
        Channel<CH5>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO1, SM1)>,
    >,
    t6: Transfer<
        Channel<CH6>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO1, SM2)>,
    >,
    t7: Transfer<
        Channel<CH7>,
        &'static mut [u32; MAX_BUFFER_WORDS],
        rp2040_hal::pio::Tx<(PIO1, SM3)>,
    >,
}

trait Wrapper {
    fn swap(&mut self, new: [u32; MAX_BUFFER_WORDS]);
}

struct Builder {
    led_us: fugit::MicrosDurationU64,
}

impl Builder {
    fn init<P: PIOExt + FunctionConfig>(
        pio: &mut PIO<P>,
    ) -> (
        InstalledProgram<P>,
        InstalledProgram<P>,
        InstalledProgram<P>,
        InstalledProgram<P>,
    ) {
        let program = init(pio);

        unsafe {
            (
                program.share(),
                program.share(),
                program.share(),
                program.share(),
            )
        }
    }

    fn common<P: PIOExt + FunctionConfig>(
        program: InstalledProgram<P>,
        int: u16,
        frac: u8,
    ) -> PIOBuilder<P> {
        rp2040_hal::pio::PIOBuilder::from_program(program)
            // only use TX FIFO
            .buffers(rp2040_hal::pio::Buffers::OnlyTx)
            // OSR config
            .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
            .autopull(true)
            .clock_divisor_fixed_point(int, frac)
    }

    fn build<
        P: PIOExt + FunctionConfig + 'static,
        A: PinId,
        B: PinId,
        C: PinId,
        D: PinId,
        E: PinId,
        F: PinId,
        G: PinId,
        H: PinId,
        W: SingleChannel + 'static,
        X: SingleChannel + 'static,
        Y: SingleChannel + 'static,
        Z: SingleChannel + 'static,
    >(
        &self,
        int: u16,
        frac: u8,
        mut split: (
            PIO<P>,
            UninitStateMachine<(P, SM0)>,
            UninitStateMachine<(P, SM1)>,
            UninitStateMachine<(P, SM2)>,
            UninitStateMachine<(P, SM3)>,
        ),
        dma: (W, X, Y, Z),
        pins: (
            Pin<A, Function<P>>,
            Pin<B, Function<P>>,
            Pin<C, Function<P>>,
            Pin<D, Function<P>>,
            Pin<E, Function<P>>,
            Pin<F, Function<P>>,
            Pin<G, Function<P>>,
            Pin<H, Function<P>>,
        ),
        bufs: (
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
        ),
    ) -> (
        Transfer<W, &'static mut [u32; MAX_BUFFER_WORDS], rp2040_hal::pio::Tx<(P, SM0)>>,
        Transfer<X, &'static mut [u32; MAX_BUFFER_WORDS], rp2040_hal::pio::Tx<(P, SM1)>>,
        Transfer<Y, &'static mut [u32; MAX_BUFFER_WORDS], rp2040_hal::pio::Tx<(P, SM2)>>,
        Transfer<Z, &'static mut [u32; MAX_BUFFER_WORDS], rp2040_hal::pio::Tx<(P, SM3)>>,
    )
    where
        Function<P>: ValidPinMode<A>,
        Function<P>: ValidPinMode<B>,
        Function<P>: ValidPinMode<C>,
        Function<P>: ValidPinMode<D>,
        Function<P>: ValidPinMode<E>,
        Function<P>: ValidPinMode<F>,
        Function<P>: ValidPinMode<G>,
        Function<P>: ValidPinMode<H>,
    {
        assert!(
            A::DYN.num == B::DYN.num - 1,
            "Provided pins must be next to each other"
        );
        assert!(
            C::DYN.num == D::DYN.num - 1,
            "Provided pins must be next to each other"
        );
        assert!(
            E::DYN.num == F::DYN.num - 1,
            "Provided pins must be next to each other"
        );
        assert!(
            G::DYN.num == H::DYN.num - 1,
            "Provided pins must be next to each other"
        );

        let (p0, p1, p2, p3) = Self::init(&mut split.0);

        let (mut sm0, _, tx0) = Self::common(p0, int, frac)
            .side_set_pin_base(pins.0.id().num)
            .build(split.1);
        sm0.set_pindirs([
            (pins.0.id().num, rp2040_hal::pio::PinDir::Output),
            (pins.1.id().num, rp2040_hal::pio::PinDir::Output),
        ]);

        let (mut sm1, _, tx1) = Self::common(p1, int, frac)
            .side_set_pin_base(C::DYN.num)
            .build(split.2);
        sm1.set_pindirs([
            (C::DYN.num, rp2040_hal::pio::PinDir::Output),
            (D::DYN.num, rp2040_hal::pio::PinDir::Output),
        ]);

        let (mut sm2, _, tx2) = Self::common(p2, int, frac)
            .side_set_pin_base(E::DYN.num)
            .build(split.3);
        sm2.set_pindirs([
            (E::DYN.num, rp2040_hal::pio::PinDir::Output),
            (F::DYN.num, rp2040_hal::pio::PinDir::Output),
        ]);

        let (mut sm3, _, tx3) = Self::common(p3, int, frac)
            .side_set_pin_base(G::DYN.num)
            .build(split.4);
        sm3.set_pindirs([
            (G::DYN.num, rp2040_hal::pio::PinDir::Output),
            (H::DYN.num, rp2040_hal::pio::PinDir::Output),
        ]);

        dma.0
            .ch()
            .ch_al1_ctrl
            .write(|w| unsafe { w.ring_size().bits(ring_size(MAX_BUFFER_SIZE)) });

        dma.1
            .ch()
            .ch_al1_ctrl
            .write(|w| unsafe { w.ring_size().bits(ring_size(MAX_BUFFER_SIZE)) });

        dma.2
            .ch()
            .ch_al1_ctrl
            .write(|w| unsafe { w.ring_size().bits(ring_size(MAX_BUFFER_SIZE)) });

        dma.3
            .ch()
            .ch_al1_ctrl
            .write(|w| unsafe { w.ring_size().bits(ring_size(MAX_BUFFER_SIZE)) });

        sm0.start();
        sm1.start();
        sm2.start();
        sm3.start();

        (
            Config::new(dma.0, bufs.0, tx0).start(),
            Config::new(dma.1, bufs.1, tx1).start(),
            Config::new(dma.2, bufs.2, tx2).start(),
            Config::new(dma.3, bufs.3, tx3).start(),
        )
    }
}

impl<T: embedded_hal::timer::CountDown> Ws2812Direct<T> {
    /// Creates a new instance of this driver.
    pub fn new<
        A: PinId + BankPinId,
        B: PinId + BankPinId,
        C: PinId + BankPinId,
        D: PinId + BankPinId,
        E: PinId + BankPinId,
        F: PinId + BankPinId,
        G: PinId + BankPinId,
        H: PinId + BankPinId,
        I: PinId + BankPinId,
        J: PinId + BankPinId,
        K: PinId + BankPinId,
        L: PinId + BankPinId,
        M: PinId + BankPinId,
        N: PinId + BankPinId,
        O: PinId + BankPinId,
        P: PinId + BankPinId,
    >(
        pio0: PIO0,
        pio1: PIO1,
        dma: Channels,
        resets: &mut RESETS,
        pins: (
            Pin<A, Function<PIO0>>,
            Pin<B, Function<PIO0>>,
            Pin<C, Function<PIO0>>,
            Pin<D, Function<PIO0>>,
            Pin<E, Function<PIO1>>,
            Pin<F, Function<PIO1>>,
            Pin<G, Function<PIO1>>,
            Pin<H, Function<PIO1>>,
            Pin<I, Function<PIO0>>,
            Pin<J, Function<PIO0>>,
            Pin<K, Function<PIO0>>,
            Pin<L, Function<PIO0>>,
            Pin<M, Function<PIO1>>,
            Pin<N, Function<PIO1>>,
            Pin<O, Function<PIO1>>,
            Pin<P, Function<PIO1>>,
        ),
        bufs: (
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
            &'static mut [u32; MAX_BUFFER_WORDS],
        ),
        clock_freq: fugit::HertzU32,
        mut timer: T,
    ) -> Self
    where
        T::Time: From<fugit::MicrosDurationU64>,
    {
        const CYCLES_PER_BIT: u32 = 16;
        const WS2812KHZ: u32 = 800;
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

        // number of us to light one LED
        let FREQ_U64: HertzU64 = HertzU64::kHz(WS2812KHZ as u64);
        let led_us = (FREQ_U64 / 24 / (10 as u32)).into_duration() + 300_u32.micros();
        info!("poop {}", led_us.to_micros());

        let b = Builder { led_us };

        let (ch0, ch1, ch2, ch3) = b.build(
            int,
            frac,
            pio0.split(resets),
            (dma.ch0, dma.ch1, dma.ch2, dma.ch3),
            (
                pins.0.into_mode(),
                pins.1.into_mode(),
                pins.2.into_mode(),
                pins.3.into_mode(),
                pins.4.into_mode(),
                pins.5.into_mode(),
                pins.6.into_mode(),
                pins.7.into_mode(),
            ),
            (bufs.0, bufs.1, bufs.2, bufs.3),
        );

        let (ch4, ch5, ch6, ch7) = b.build(
            int,
            frac,
            pio1.split(resets),
            (dma.ch4, dma.ch5, dma.ch6, dma.ch7),
            (
                pins.8.into_mode(),
                pins.9.into_mode(),
                pins.10.into_mode(),
                pins.11.into_mode(),
                pins.12.into_mode(),
                pins.13.into_mode(),
                pins.14.into_mode(),
                pins.15.into_mode(),
            ),
            (bufs.4, bufs.5, bufs.6, bufs.7),
        );

        timer.start(led_us);

        Self {
            timer: timer,
            led_us: led_us,
            t0: ch0,
            t1: ch1,
            t2: ch2,
            t3: ch3,
            t4: ch4,
            t5: ch5,
            t6: ch6,
            t7: ch7,
        }
    }

    pub fn write<K: Into<RGB8> + Clone + Copy, const COUNT: usize>(
        mut self,
        leds: &[LEDs<COUNT, K>; 8],
    ) -> Self
    where
        T::Time: From<fugit::MicrosDurationU64>,
    {
        let (a, b, c) = self.t0.wait();
        block!(self.timer.wait()).unwrap();

        leds[0].fill(b);
        self.t0 = Config::new(a, b, c).start();
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

// struct Foo {
//     bufuser: Bar,
//     buf: [u8; 8],
// }

// struct Bar {
//     buf: &[u8; 8],
// }

// fn new_foo() -> Foo {}

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
