//! Asynchronous Test Executor

#![no_std]
#![feature(alloc_error_handler)]

extern crate alloc;

pub use alloc_cortex_m;

use alloc::{boxed::Box, collections::BTreeMap, sync::Arc, task::Wake};
use alloc_cortex_m::CortexMHeap;
use core::{
    future::Future,
    pin::Pin,
    sync::atomic::{AtomicU32, Ordering},
    task::{Context, Poll, Waker},
};
use crossbeam_queue::ArrayQueue;
use hal::{gpio, pac};
use stm32wl_hal as hal;

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_layout: core::alloc::Layout) -> ! {
    hal::cortex_m::interrupt::disable();

    let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let mut rcc: pac::RCC = dp.RCC;

    let gpiob: gpio::PortB = gpio::PortB::split(dp.GPIOB, &mut rcc);
    let mut led1 = gpio::Output::default(gpiob.pb9);
    let mut led2 = gpio::Output::default(gpiob.pb15);
    let mut led3 = gpio::Output::default(gpiob.pb11);

    led1.set_level_high();
    led2.set_level_high();
    led3.set_level_high();

    use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
    loop {
        compiler_fence(SeqCst);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct TaskId(u32);

impl TaskId {
    fn new() -> Self {
        static NEXT_ID: AtomicU32 = AtomicU32::new(0);
        TaskId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

pub struct Task {
    id: TaskId,
    future: Pin<Box<dyn Future<Output = ()>>>,
}

impl Task {
    pub fn new(future: impl Future<Output = ()> + 'static) -> Task {
        Task {
            id: TaskId::new(),
            future: Box::pin(future),
        }
    }

    fn poll(&mut self, context: &mut Context) -> Poll<()> {
        self.future.as_mut().poll(context)
    }
}

struct TaskWaker {
    task_id: TaskId,
    task_queue: Arc<ArrayQueue<TaskId>>,
}

impl TaskWaker {
    #[allow(clippy::new_ret_no_self)]
    fn new(task_id: TaskId, task_queue: Arc<ArrayQueue<TaskId>>) -> Waker {
        Waker::from(Arc::new(TaskWaker {
            task_id,
            task_queue,
        }))
    }

    fn wake_task(&self) {
        self.task_queue.push(self.task_id).expect("task_queue full");
    }
}

impl Wake for TaskWaker {
    fn wake(self: Arc<Self>) {
        self.wake_task();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.wake_task();
    }
}

pub struct Executor {
    tasks: BTreeMap<TaskId, Task>,
    task_queue: Arc<ArrayQueue<TaskId>>,
    waker_cache: BTreeMap<TaskId, Waker>,
}

impl Executor {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Executor {
        Executor {
            tasks: BTreeMap::new(),
            task_queue: Arc::new(ArrayQueue::new(64)),
            waker_cache: BTreeMap::new(),
        }
    }

    pub fn spawn(&mut self, task: Task) {
        let task_id = task.id;
        if self.tasks.insert(task.id, task).is_some() {
            panic!("task with same ID already in tasks");
        }
        self.task_queue.push(task_id).expect("queue full");
    }

    fn run_ready_tasks(&mut self) {
        // destructure self to avoid borrow checker errors
        // https://github.com/rust-lang/rust/issues/53488
        let Self {
            tasks,
            task_queue,
            waker_cache,
        } = self;

        while let Some(task_id) = task_queue.pop() {
            let task = match tasks.get_mut(&task_id) {
                Some(task) => task,
                None => continue, // task no longer exists
            };
            let waker = waker_cache
                .entry(task_id)
                .or_insert_with(|| TaskWaker::new(task_id, task_queue.clone()));
            let mut context = Context::from_waker(waker);
            match task.poll(&mut context) {
                Poll::Ready(()) => {
                    // task done -> remove it and its cached waker
                    tasks.remove(&task_id);
                    waker_cache.remove(&task_id);
                }
                Poll::Pending => {}
            }
        }
    }

    pub fn run(&mut self) {
        loop {
            self.run_ready_tasks();
            if self.tasks.is_empty() {
                break;
            }
        }
    }
}
