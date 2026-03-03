use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(1);

pub fn unique_topic(prefix: &str) -> String {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{prefix}_{id}")
}

#[allow(dead_code)]
pub fn wait_until(timeout: Duration, step: Duration, mut condition: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if condition() {
            return true;
        }
        std::thread::sleep(step);
    }
    condition()
}
