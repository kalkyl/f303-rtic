// $ cargo rb serial
#![no_main]
#![no_std]

use f303_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [WWDG])]
mod app {
    use stm32f3xx_hal::{
        dma::{
            dma1::{C4, C5},
            Channel, Event, Transfer,
        },
        pac::USART1,
        prelude::*,
        serial::{Rx, Serial, Tx},
    };
    const BUF_SIZE: usize = 12;

    pub enum TxTransfer {
        Running(Transfer<&'static mut [u8; BUF_SIZE], C4, Tx<USART1>>),
        Idle(&'static mut [u8; BUF_SIZE], C4, Tx<USART1>),
    }

    #[shared]
    struct Shared {
        #[lock_free]
        send: Option<TxTransfer>,
    }

    #[local]
    struct Local {
        recv: Option<Transfer<&'static mut [u8; BUF_SIZE], C5, Rx<USART1>>>,
    }

    #[init(local = [tx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE], rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());
        let mut rcc = ctx.device.RCC.constrain();
        let mut flash = ctx.device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc.ahb);
        let pins = (
            gpioc
                .pc4
                .into_af7_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl),
            gpioc
                .pc5
                .into_af7_push_pull(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl),
        );
        let serial = Serial::new(ctx.device.USART1, pins, 9600.Bd(), clocks, &mut rcc.apb2);
        let (tx, rx) = serial.split();
        let mut dma1 = ctx.device.DMA1.split(&mut rcc.ahb);
        dma1.ch4.listen(Event::TransferComplete);
        dma1.ch5.listen(Event::TransferComplete);
        (
            Shared {
                send: Some(TxTransfer::Idle(ctx.local.tx_buf, dma1.ch4, tx)),
            },
            Local {
                recv: Some(rx.read_exact(ctx.local.rx_buf, dma1.ch5)),
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    // Triggers on RX transfer completed
    #[task(binds = DMA1_CH5, local = [recv], priority = 2)]
    fn on_rx(ctx: on_rx::Context) {
        let (rx_buf, rx_channel, rx) = ctx.local.recv.take().unwrap().wait();
        echo::spawn(*rx_buf).ok();
        ctx.local.recv.replace(rx.read_exact(rx_buf, rx_channel));
    }

    #[task(shared = [send], priority = 1, capacity = 4)]
    fn echo(ctx: echo::Context, data: [u8; BUF_SIZE]) {
        defmt::info!("Received {:?}", data);
        let send = ctx.shared.send;
        let (tx_buf, tx_channel, tx) = match send.take().unwrap() {
            TxTransfer::Idle(buf, ch, tx) => (buf, ch, tx),
            TxTransfer::Running(transfer) => transfer.wait(),
        };
        tx_buf.copy_from_slice(&data[..]);
        send.replace(TxTransfer::Running(tx.write_all(tx_buf, tx_channel)));
    }

    // Triggers on TX transfer completed
    #[task(binds = DMA1_CH4, shared = [send], priority = 1)]
    fn on_tx(ctx: on_tx::Context) {
        let send = ctx.shared.send;
        let (tx_buf, tx_channel, tx) = match send.take().unwrap() {
            TxTransfer::Idle(buf, ch, tx) => (buf, ch, tx),
            TxTransfer::Running(transfer) => transfer.wait(),
        };
        defmt::info!("Sent {:?}", tx_buf);
        send.replace(TxTransfer::Idle(tx_buf, tx_channel, tx));
    }
}
