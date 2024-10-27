//! Receive bytes over nRF24L01.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m_rt::entry;
use heapless::spsc::Queue;
use nrf24l01_commands::{commands, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32u5::stm32u575::{interrupt, Interrupt, Peripherals, EXTI, GPDMA1, GPIOA, SPI1, USART2};

const RX_ADDR: u64 = 0xA2891FFF6A;
const SPI1_TXDR: u32 = 0x4001_3020;
const SPI1_RXDR: u32 = 0x4001_3030;
const USART2_TDR: u32 = 0x4000_4428;

// nRF24L01 command byte sequences
const W_RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(110)).bytes();
const R_RF_CH: [u8; 2] = commands::RRegister::<registers::RfCh>::bytes();
const W_RX_ADDR_P0: [u8; 6] =
    commands::WRegister(registers::RxAddrP0::<5>::new().with_rx_addr_p0(RX_ADDR)).bytes();
const R_RX_ADDR_P0: [u8; 6] = commands::RRegister::<registers::RxAddrP0<5>>::bytes();
const W_RX_PW_P0: [u8; 2] = commands::WRegister(registers::RxPwP0::new().with_rx_pw_p0(32)).bytes();
const R_RX_PW_P0: [u8; 2] = commands::RRegister::<registers::RxPwP0>::bytes();
const W_CONFIG: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_prim_rx(true)
        .with_pwr_up(true)
        .with_mask_max_rt(true)
        .with_mask_tx_ds(true),
)
.bytes();
const R_CONFIG: [u8; 2] = commands::RRegister::<registers::Config>::bytes();
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
    fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncPeripheral<GPIOA> {}
unsafe impl Sync for SyncPeripheral<USART2> {}
unsafe impl Sync for SyncPeripheral<SPI1> {}
unsafe impl Sync for SyncPeripheral<EXTI> {}
unsafe impl Sync for SyncPeripheral<GPDMA1> {}

static GPIOA_PERIPHERAL: SyncPeripheral<GPIOA> = SyncPeripheral::new();
static USART2_PERIPHERAL: SyncPeripheral<USART2> = SyncPeripheral::new();
static SPI1_PERIPHERAL: SyncPeripheral<SPI1> = SyncPeripheral::new();
static EXTI_PERIPHERAL: SyncPeripheral<EXTI> = SyncPeripheral::new();
static GPDMA1_PERIPHERAL: SyncPeripheral<GPDMA1> = SyncPeripheral::new();

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
unsafe impl Sync for SyncQueue<&[u8], 16> {}

/// Queue of commands
static COMMANDS: SyncQueue<&[u8], 16> = SyncQueue::new();
/// Byte buffer for SPI1 RX
static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];

fn send_command(command: &[u8], gpdma1: &mut GPDMA1, spi1: &mut SPI1) {
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
    gpdma1
        .c1dar()
        .write(|w| unsafe { w.bits(SPI1_RX_BUFFER.as_ptr() as u32) });
    gpdma1
        .c1br1()
        .write(|w| unsafe { w.bndt().bits(transfer_size) });
    gpdma1.c1cr().modify(|_, w| w.en().set_bit());

    // USART2 TX: Re-configure source address, transfer size
    #[allow(static_mut_refs)]
    gpdma1
        .c2sar()
        .write(|w| unsafe { w.bits(SPI1_RX_BUFFER.as_ptr() as u32) });
    gpdma1
        .c2br1()
        .write(|w| unsafe { w.bndt().bits(transfer_size) });

    // SPI1 TX: Re-configure destination address, transfer size
    gpdma1
        .c0sar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    gpdma1
        .c0br1()
        .write(|w| unsafe { w.bndt().bits(transfer_size) });
    gpdma1.c0cr().write(|w| w.en().set_bit());

    // Enable SPI
    spi1.spi_cr2()
        .write(|w| unsafe { w.tsize().bits(transfer_size) });
    spi1.spi_cr1().modify(|_, w| w.spe().set_bit());
    spi1.spi_cr1().modify(|_, w| w.cstart().set_bit());
}

// #[interrupt]
// fn USART2() {
//     let usart2 = USART2_PERIPHERAL.get();
//     let spi1 = SPI1_PERIPHERAL.get();
//     let gpioa = GPIOA_PERIPHERAL.get();

//     // Dequeue bytes off rx buffer and transmit over USART2
//     if usart2.isr_disabled().read().txfnf().bit_is_set() {
//         if let Some(byte) = rx_buffer.dequeue() {
//             usart2.tdr().write(|w| unsafe { w.tdr().bits(byte) });
//             if rx_buffer.is_empty() {
//                 usart2.cr1_disabled().modify(|_, w| w.txfnfie().clear_bit());
//             }
//         } else {
//             usart2.cr1_disabled().modify(|_, w| w.txfnfie().clear_bit());
//         }
//     }

//     // Read incoming bytes from USART2 run nRF24L01 command
//     if usart2.isr_disabled().read().rxfne().bit_is_set() {
//         // Read data, this clears RXNE
//         let received_byte = usart2.rdr().read().rdr().bits();

//         match received_byte {
//             97 => {
//                 // a
//                 // NOP
//                 send_command(&[commands::Nop::WORD], spi1, usart2, rx_buffer);
//             }
//             98 => {
//                 // b
//                 // Read RF_CH
//                 send_command(
//                     &commands::RRegister::<registers::RfCh>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             99 => {
//                 // c
//                 // Read RX Addr P0
//                 send_command(
//                     &commands::RRegister::<registers::RxAddrP0<5>>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             100 => {
//                 // d
//                 // Read Config
//                 send_command(
//                     &commands::RRegister::<registers::Config>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             101 => {
//                 // e
//                 // Clear RX_DR flag
//                 send_command(
//                     &commands::WRegister(registers::Status::new().with_rx_dr(true)).bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             102 => {
//                 // f
//                 // Read pipe 0 payload width
//                 send_command(
//                     &commands::RRegister::<registers::RxPwP0>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             103 => {
//                 // g
//                 // Read FifoStatus
//                 send_command(
//                     &commands::RRegister::<registers::FifoStatus>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             104 => {
//                 // h
//                 // CE=1
//                 gpioa.bsrr().write(|w| w.bs0().set_bit());
//             }
//             105 => {
//                 // i
//                 // CE=0
//                 gpioa.bsrr().write(|w| w.br0().set_bit());
//             }
//             106 => {
//                 // j
//                 // read payload
//                 send_command(
//                     &commands::RRxPayload::<32>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             107 => {
//                 // k
//                 // carrier detect
//                 send_command(
//                     &commands::RRegister::<registers::Cd>::bytes(),
//                     spi1,
//                     usart2,
//                     rx_buffer,
//                 );
//             }
//             _ => (),
//         }
//     }
//     if usart2.isr_disabled().read().ore().bit_is_set() {
//         usart2.icr().write(|w| w.orecf().set_bit());
//     }
// }

#[interrupt]
fn EXTI1() {
    let gpdma1 = GPDMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let exti = EXTI_PERIPHERAL.get();
    let commands = COMMANDS.get();

    if exti.fpr1().read().fpif1().bit_is_set() {
        let _ = commands.enqueue(&W_RESET_RX_DR);
        send_command(&R_RX_PAYLOAD, gpdma1, spi1);

        exti.fpr1().write(|w| w.fpif1().clear_bit_by_one());
    }
}

/// SPI1 RX DMA transfer complete
#[interrupt]
fn GPDMA1_CH1() {
    let gpdma1 = GPDMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();

    if gpdma1.c1sr().read().tcf().bit_is_set() {
        gpdma1.c1fcr().write(|w| w.tcf().set_bit());

        // Reset SPI flags
        spi1.spi_ifcr().write(|w| w.txtfc().set_bit());
        spi1.spi_cr1().modify(|_, w| w.spe().clear_bit());

        // Enable USART2 TX DMA
        gpdma1.c2cr().modify(|_, w| w.en().set_bit());
    }
}

/// USART2 TX DMA transfer complete
#[interrupt]
fn GPDMA1_CH2() {
    let gpdma1 = GPDMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let commands = COMMANDS.get();

    if gpdma1.c2sr().read().tcf().bit_is_set() {
        gpdma1.c2fcr().write(|w| w.tcf().set_bit());

        // Reset USART2 flags
        while usart2.isr_disabled().read().tc().bit_is_clear() {}
        usart2.icr().write(|w| w.tccf().set_bit());

        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, gpdma1, spi1);
        }
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = Peripherals::take().unwrap();

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
    let _ = commands.enqueue(&R_RF_CH);
    let _ = commands.enqueue(&W_RX_ADDR_P0);
    let _ = commands.enqueue(&R_RX_ADDR_P0);
    let _ = commands.enqueue(&W_RX_PW_P0);
    let _ = commands.enqueue(&R_RX_PW_P0);
    let _ = commands.enqueue(&W_CONFIG);
    let _ = commands.enqueue(&R_CONFIG);

    // USART2 TX DMA stream
    dp.GPDMA1
        .c2tr1()
        .write(|w| w.sap().set_bit().sinc().set_bit());
    dp.GPDMA1.c2tr2().write(|w| unsafe { w.reqsel().bits(27) });
    dp.GPDMA1.c2dar().write(|w| unsafe { w.bits(USART2_TDR) });
    dp.GPDMA1.c2cr().write(|w| w.tcie().set_bit());

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

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417
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
        .cr1_disabled()
        .write(|w| w.re().set_bit().te().set_bit().ue().set_bit());

    // Set up EXTI line 1 interrupt for A1
    dp.EXTI.exticr1().write(|w| w.exti1().pa());
    dp.EXTI.ftsr1().write(|w| w.ft1().set_bit());
    dp.EXTI.imr1().write(|w| w.im1().set_bit());

    GPDMA1_PERIPHERAL.set(dp.GPDMA1);
    GPIOA_PERIPHERAL.set(dp.GPIOA);
    SPI1_PERIPHERAL.set(dp.SPI1);
    USART2_PERIPHERAL.set(dp.USART2);
    EXTI_PERIPHERAL.set(dp.EXTI);
    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::GPDMA1_CH1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::GPDMA1_CH2);
    }

    // Begin transfer for initial command
    let gpdma1 = GPDMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();

    send_command(&W_RF_CH, gpdma1, spi1);

    gpioa.bsrr().write(|w| w.bs0().set_bit());

    #[allow(clippy::empty_loop)]
    loop {}
}
