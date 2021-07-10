#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use alloc_cortex_m::CortexMHeap;
use core::{
    alloc::Layout,
    fmt::Write,
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};
use examples::aio::{Executor, Task};
use rtt_target::rprintln;
use stm32wl_hal::{
    aes::{Aes, Key, Key128},
    pac,
};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    if let Some(mut channel) = unsafe { rtt_target::UpChannel::conjure(0) } {
        channel.set_mode(rtt_target::ChannelMode::BlockIfFull);

        writeln!(channel, "{}", info).ok();
    }

    loop {
        compiler_fence(SeqCst);
    }
}

#[alloc_error_handler]
fn oom(layout: Layout) -> ! {
    cortex_m::interrupt::disable();

    if let Some(mut channel) = unsafe { rtt_target::UpChannel::conjure(0) } {
        channel.set_mode(rtt_target::ChannelMode::BlockIfFull);

        writeln!(channel, "{:?}", layout).ok();
    }

    loop {
        compiler_fence(SeqCst);
    }
}

pub const fn u128_to_u32(u: u128) -> [u32; 4] {
    [
        (u >> 96) as u32,
        (u >> 64) as u32,
        (u >> 32) as u32,
        u as u32,
    ]
}

// NIST AESAVS Nov 15 2002 B.1
const PLAINTEXT_CHIPHERTEXT: [(u128, u128); 7] = [
    (
        0xf34481ec3cc627bacd5dc3fb08f273e6,
        0x0336763e966d92595a567cc9ce537f5e,
    ),
    (
        0x9798c4640bad75c7c3227db910174e72,
        0xa9a1631bf4996954ebc093957b234589,
    ),
    (
        0x96ab5c2ff612d9dfaae8c31f30c42168,
        0xff4f8391a6a40ca5b25d23bedd44a597,
    ),
    (
        0x6a118a874519e64e9963798a503f1d35,
        0xdc43be40be0e53712f7e2bf5ca707209,
    ),
    (
        0xcb9fceec81286ca3e989bd979b0cb284,
        0x92beedab1895a94faa69b632e5cc47ce,
    ),
    (
        0xb26aeb1874e47ca8358ff22378f09144,
        0x459264f4798f6a78bacb89c15ed3d601,
    ),
    (
        0x58c8e00b2631686d54eab84b91f0aca1,
        0x08a4e2efec8a8e3312ca7460b9040bbf,
    ),
];

async fn aes_task(mut aes: Aes) {
    rprintln!("[AES] decrypt start");
    const KEY: Key = Key::K128(Key128::from_u128(0));
    for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
        let result: [u32; 4] = aes
            .aio_decrypt_ecb(&KEY, &u128_to_u32(*ciphertext))
            .await
            .unwrap();
        let plaintext_u32: [u32; 4] = u128_to_u32(*plaintext);
        assert!(
            result == plaintext_u32,
            "\nResult:   {:08X?}\nExpected: {:08X?}\n",
            result,
            plaintext_u32
        );
    }
    rprintln!("[AES] decrypt done");
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 4096
                mode: BlockIfFull
                name: "Terminal"
            }
        }
    };

    writeln!(&mut channels.up.0, "Hello from writeln!").ok();

    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Hello from rprintln!");

    // Initialize the allocator BEFORE you use it
    let start: usize = cortex_m_rt::heap_start() as usize;
    let size: usize = 2048; // in bytes
    rprintln!("[heap] init at 0x{:X} size {}B", start, size);
    unsafe { ALLOCATOR.init(start, size) };
    rprintln!("[heap] init done");

    let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC;

    rcc.ahb3enr.modify(|_, w| w.aesen().set_bit());
    rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling

    unsafe { pac::NVIC::unmask(pac::Interrupt::AES) };

    rprintln!("[AES] init");
    let aes: Aes = Aes::new(dp.AES, &mut rcc);
    rprintln!("[AES] init done");

    rprintln!("[AIO] executor start");
    let mut executor = Executor::new();
    executor.spawn(Task::new(aes_task(aes)));
    executor.run();
    rprintln!("[AIO] executor done");

    loop {
        compiler_fence(SeqCst);
    }
}
