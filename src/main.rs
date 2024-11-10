//! Receive bytes over nRF24L01.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m::asm;
use cortex_m_rt::entry;
use heapless::spsc::Queue;
use nrf24l01_commands::{commands, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32u5::stm32u575::{interrupt, Interrupt, Peripherals};

const RX_ADDR: u64 = 0xA2891F;
const SPI1_TXDR: u32 = 0x4001_3020;
const SPI1_RXDR: u32 = 0x4001_3030;
const USART2_TDR: u32 = 0x4000_4428;

// nRF24L01 command byte sequences
// const NOP: [u8; 1] = commands::Nop::bytes();
const W_RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(0)).bytes();
const W_RF_SETUP: [u8; 2] =
    commands::WRegister(registers::RfSetup::new().with_rf_dr(false)).bytes();
const W_SETUP_AW: [u8; 2] = commands::WRegister(registers::SetupAw::new().with_aw(1)).bytes();
const W_RX_ADDR_P0: [u8; 4] =
    commands::WRegister(registers::RxAddrP0::<3>::new().with_rx_addr_p0(RX_ADDR)).bytes();
const W_RX_PW_P0: [u8; 2] = commands::WRegister(registers::RxPwP0::new().with_rx_pw_p0(32)).bytes();
const W_CONFIG: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_prim_rx(true)
        .with_pwr_up(true)
        .with_mask_max_rt(true)
        .with_mask_tx_ds(true),
)
.bytes();
const R_RX_PAYLOAD: [u8; 33] = commands::RRxPayload::<32>::bytes();
const W_RESET_RX_DR: [u8; 2] =
    commands::WRegister(registers::Status::new().with_rx_dr(true)).bytes();

struct SyncPeripheral<P>(UnsafeCell<Option<P>>);

impl<P> SyncPeripheral<P> {
    const fn new() -> Self {
        SyncPeripheral(UnsafeCell::new(None))
    }

    fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    #[allow(clippy::mut_from_ref)]
    const fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncPeripheral<Peripherals> {}

static DEVICE_PERIPHERALS: SyncPeripheral<Peripherals> = SyncPeripheral::new();

struct SyncQueue<T, const N: usize>(UnsafeCell<Queue<T, N>>);

impl<T, const N: usize> SyncQueue<T, N> {
    const fn new() -> Self {
        Self(UnsafeCell::new(Queue::new()))
    }

    #[allow(clippy::mut_from_ref)]
    const fn get(&self) -> &mut Queue<T, N> {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncQueue<&[u8], 8> {}

/// Queue of commands
static COMMANDS: SyncQueue<&[u8], 8> = SyncQueue::new();
/// Byte buffer for SPI1 RX
static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];

fn send_command(command: &[u8], dp: &mut Peripherals) {
    /* SPI Transmit procedure:
       TSIZE must be written before SPE=1, p.2924: SPI must be disabled while changing TSIZE
       SPE=1
       Push bytes to TXFIFO through TXDR, p.2908: FIFOs cannot be accessed with SPE=0
       TXTF gets set after bytes are written to TXFIFO, clear it
       CSTART=1
       Poll EOT=1, CSTART is reset when EOT is set
       Read TSIZE bytes out of RXFIFO
       SPE=0
    */

    let transfer_size = command.len() as u16;

    // SPI1 RX: Re-configure destination address, transfer size
    #[allow(static_mut_refs)]
    dp.GPDMA1
        .c1dar()
        .write(|w| unsafe { w.bits(SPI1_RX_BUFFER.as_ptr() as u32) });
    dp.GPDMA1
        .c1br1()
        .write(|w| unsafe { w.bndt().bits(transfer_size) });
    dp.GPDMA1.c1cr().modify(|_, w| w.en().set_bit());

    // USART2 TX: Re-configure source address, transfer size
    if command == R_RX_PAYLOAD {
        #[allow(static_mut_refs)]
        dp.GPDMA1
            .c2sar()
            .write(|w| unsafe { w.bits(SPI1_RX_BUFFER[1..].as_ptr() as u32) });
        dp.GPDMA1.c2br1().write(|w| unsafe { w.bndt().bits(32) });
    }

    // SPI1 TX: Re-configure destination address, transfer size
    dp.GPDMA1
        .c0sar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    dp.GPDMA1
        .c0br1()
        .write(|w| unsafe { w.bndt().bits(transfer_size) });
    dp.GPDMA1.c0cr().write(|w| w.en().set_bit());

    // Enable SPI
    dp.SPI1
        .spi_cr2()
        .write(|w| unsafe { w.tsize().bits(transfer_size) });
    dp.SPI1.spi_cr1().modify(|_, w| w.spe().set_bit());
    dp.SPI1.spi_cr1().modify(|_, w| w.cstart().set_bit());
}

#[interrupt]
fn EXTI1() {
    let dp = DEVICE_PERIPHERALS.get();
    let commands = COMMANDS.get();

    if dp.EXTI.fpr1().read().fpif1().bit_is_set() {
        unsafe {
            commands.enqueue_unchecked(&W_RESET_RX_DR);
        }
        send_command(&R_RX_PAYLOAD, dp);

        dp.EXTI.fpr1().write(|w| w.fpif1().clear_bit_by_one());
    }
}

/// SPI1 RX DMA transfer complete
#[interrupt]
fn GPDMA1_CH1() {
    let dp = DEVICE_PERIPHERALS.get();
    let commands = COMMANDS.get();

    if dp.GPDMA1.c1sr().read().tcf().bit_is_set() {
        dp.GPDMA1.c1fcr().write(|w| w.tcf().set_bit());

        // Reset SPI flags
        dp.SPI1.spi_ifcr().write(|w| w.txtfc().set_bit());
        dp.SPI1.spi_cr1().modify(|_, w| w.spe().clear_bit());

        // USART transaction will complete before next payload is received
        if dp.GPDMA1.c0sar().read().sa() == R_RX_PAYLOAD.as_ptr() as u32 + 33 {
            // Enable USART2 TX DMA if payload was read
            while dp.USART2.isr_enabled().read().tc().bit_is_clear() {}
            dp.USART2.icr().write(|w| w.tccf().set_bit());
            dp.GPDMA1.c2cr().modify(|_, w| w.en().set_bit());
        }
        if let Some(command) = commands.dequeue() {
            send_command(command, dp);
        }
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    unsafe {
        // Set SLEEPONEXIT
        cp.SCB.scr.write(0b10);
    }

    // Enable peripheral clocks - GPDMA1, GPIOA, USART2, SPI1
    dp.RCC.ahb1enr().write(|w| w.gpdma1en().set_bit());
    dp.RCC.ahb2enr1().write(|w| w.gpioaen().enabled());
    dp.RCC.apb1enr1().write(|w| w.usart2en().enabled());
    dp.RCC.apb2enr().write(|w| w.spi1en().enabled());

    // GPIO: A1 (IRQ), A0 (CE)
    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    dp.GPIOA.moder().write(|w| {
        w.mode0()
            .output()
            .mode1()
            .input()
            .mode2()
            .alternate()
            .mode3()
            .alternate()
            .mode4()
            .alternate()
            .mode5()
            .alternate()
            .mode6()
            .alternate()
            .mode7()
            .alternate()
    });
    dp.GPIOA.otyper().write(|w| w.ot0().push_pull());
    // NSS, IRQ are active low
    dp.GPIOA
        .pupdr()
        .write(|w| w.pupd1().pull_up().pupd4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeed2()
            .medium_speed()
            .ospeed3()
            .medium_speed()
            .ospeed4()
            .medium_speed()
            .ospeed5()
            .medium_speed()
            .ospeed6()
            .medium_speed()
            .ospeed7()
            .medium_speed()
    });
    dp.GPIOA.afrl().write(|w| {
        w.afsel2()
            .af7()
            .afsel3()
            .af7()
            .afsel4()
            .af5()
            .afsel5()
            .af5()
            .afsel6()
            .af5()
            .afsel7()
            .af5()
    });

    // Enqueue initialization commands
    let commands = COMMANDS.get();
    unsafe {
        commands.enqueue_unchecked(&W_RF_SETUP);
        commands.enqueue_unchecked(&W_SETUP_AW);
        commands.enqueue_unchecked(&W_RX_ADDR_P0);
        commands.enqueue_unchecked(&W_RX_PW_P0);
        commands.enqueue_unchecked(&W_CONFIG);
    }

    // USART2 TX DMA stream
    dp.GPDMA1
        .c2tr1()
        .write(|w| w.sap().set_bit().sinc().set_bit());
    dp.GPDMA1.c2tr2().write(|w| unsafe { w.reqsel().bits(27) });
    dp.GPDMA1.c2dar().write(|w| unsafe { w.bits(USART2_TDR) });

    // SPI1 RX DMA stream
    dp.GPDMA1
        .c1tr1()
        .write(|w| w.dap().set_bit().dinc().set_bit());
    dp.GPDMA1.c1tr2().write(|w| unsafe { w.reqsel().bits(6) });
    dp.GPDMA1.c1sar().write(|w| unsafe { w.bits(SPI1_RXDR) });
    dp.GPDMA1.c1cr().write(|w| w.tcie().set_bit());

    // SPI1 TX DMA stream
    dp.GPDMA1
        .c0tr1()
        .write(|w| w.sap().set_bit().sinc().set_bit());
    dp.GPDMA1.c0tr2().write(|w| unsafe { w.reqsel().bits(7) });
    dp.GPDMA1.c0dar().write(|w| unsafe { w.bits(SPI1_TXDR) });

    // USART2: Configure baud rate 115200
    dp.USART2.brr().write(|w| unsafe { w.bits(35) }); // 4Mhz / 115200 approx. 35
    dp.USART2.cr3().write(|w| w.dmat().set_bit());

    // SPI1: Enable DMA, set SPI master, enable hardware NSS
    dp.SPI1
        .spi_cfg1()
        .write(|w| w.txdmaen().set_bit().rxdmaen().set_bit());
    dp.SPI1
        .spi_cfg2()
        .write(|w| w.ssoe().set_bit().master().set_bit());

    // Enable USART, transmitter and receiver
    dp.USART2
        .cr1_enabled()
        .write(|w| w.fifoen().set_bit().te().set_bit().ue().set_bit());

    // Set up EXTI line 1 interrupt for A1
    dp.EXTI.exticr1().write(|w| w.exti1().pa());
    dp.EXTI.ftsr1().write(|w| w.ft1().set_bit());
    dp.EXTI.imr1().write(|w| w.im1().set_bit());

    DEVICE_PERIPHERALS.set(dp);
    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::GPDMA1_CH1);
    }

    // Begin transfer for initial command
    let dp = DEVICE_PERIPHERALS.get();

    send_command(&W_RF_CH, dp);

    dp.GPIOA.bsrr().write(|w| w.bs0().set_bit());

    loop {
        asm::wfi();
    }
}
