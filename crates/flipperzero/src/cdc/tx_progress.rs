//! USB endpoint completion state, independent of firmware for host tests.
use core::sync::atomic::{AtomicU8, Ordering};

const IDLE: u8 = 0;
const SHORT: u8 = 1;
const FULL: u8 = 2;
const ZLP: u8 = 3;
const INITIAL: u8 = 4;
const DISCONNECTED: u8 = 5;

pub(super) struct TxProgress(AtomicU8);

impl TxProgress {
    pub(super) fn new(enumerating: bool) -> Self {
        Self(AtomicU8::new(if enumerating { INITIAL } else { IDLE }))
    }
    pub(super) fn idle(&self) -> bool {
        self.0.load(Ordering::Acquire) == IDLE
    }
    pub(super) fn disconnected(&self) -> bool {
        self.0.load(Ordering::Acquire) == DISCONNECTED
    }
    // Caller excludes USB interrupts between this claim and endpoint submission.
    pub(super) fn begin(&self, full: bool) -> bool {
        self.0
            .compare_exchange(
                IDLE,
                if full { FULL } else { SHORT },
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }
    // Return true only after a full data packet completes: submit its terminator.
    pub(super) fn complete(&self) -> bool {
        let state = self.0.load(Ordering::Acquire);
        let next = match state {
            FULL => ZLP,
            INITIAL | SHORT | ZLP => IDLE,
            _ => return false,
        };
        self.0
            .compare_exchange(state, next, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
            && state == FULL
    }
    pub(super) fn suspend(&self) {
        // Initial mode-switch suspend has not submitted application data yet.
        if self.0.load(Ordering::Acquire) != INITIAL {
            self.0.store(DISCONNECTED, Ordering::Release);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn enumeration_completion_does_not_complete_application_data() {
        let tx = TxProgress::new(true);
        assert!(!tx.begin(false));
        assert!(!tx.complete());
        assert!(tx.idle());
        assert!(tx.begin(false));
        assert!(!tx.idle());
        assert!(!tx.complete());
        assert!(tx.idle());
    }
    #[test]
    fn full_packet_requires_data_and_zlp_completions() {
        let tx = TxProgress::new(false);
        assert!(tx.begin(true));
        assert!(!tx.begin(false));
        assert!(tx.complete());
        assert!(!tx.idle());
        assert!(!tx.begin(false));
        assert!(!tx.complete());
        assert!(tx.idle());
        assert!(!tx.complete()); // stray completion does not emit another ZLP
    }
    #[test]
    fn pending_state_survives_timeout_and_late_completion() {
        let tx = TxProgress::new(false);
        assert!(tx.begin(true));
        // A deadline only returns an error; it must not reset this state.
        assert!(!tx.idle());
        assert!(!tx.begin(false));
        assert!(tx.complete());
        assert!(!tx.idle());
        assert!(!tx.complete());
        assert!(tx.begin(false));
    }
    #[test]
    fn disconnect_is_sticky_until_session_recreated() {
        let tx = TxProgress::new(false);
        assert!(tx.begin(true));
        tx.suspend();
        assert!(tx.disconnected());
        assert!(!tx.complete());
        assert!(!tx.idle());
        assert!(!tx.begin(false));
    }
    #[test]
    fn initial_mode_switch_suspend_is_allowed() {
        let tx = TxProgress::new(true);
        tx.suspend();
        assert!(!tx.disconnected());
        assert!(!tx.complete());
        assert!(tx.idle());
    }
}
