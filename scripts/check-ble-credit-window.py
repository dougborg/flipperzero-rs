#!/usr/bin/env python3
"""Exhaust the interleavings of a credit-honoring sender and read/notify.

The service decrements credits before queuing received data. Notify grants a
whole new window only when the remaining credits are zero. The application's
empty observation happens outside that mutex. Model one pending notify, since
all application I/O is exclusive. Queue capacity must cover a raced old window
plus the new grant; the original one-window allocation has a counterexample.
"""
from collections import deque


def explore(window):
    initial = (0, window, False)
    queue = deque([initial])
    seen = {initial}
    maximum = 0
    while queue:
        buffered, credits, notifying = queue.popleft()
        maximum = max(maximum, buffered)
        successors = []
        # A compliant central never exceeds the last advertised grant.
        for received in range(1, credits + 1):
            successors.append((buffered + received, credits - received, notifying))
        if notifying:
            successors.append((buffered, window if credits == 0 else credits, False))
        else:
            for consumed in range(1, buffered + 1):
                successors.append((buffered - consumed, credits, False))
            if buffered == 0:
                successors.append((0, credits, True))
        for state in successors:
            assert state[0] <= 2 * window, (window, state)
            if state not in seen:
                seen.add(state)
                queue.append(state)
    assert maximum == 2 * window, 'must reproduce the original one-window overflow'
    return len(seen)


for size in range(1, 17):
    print(f'credits={size}: {explore(size)} states; two windows sufficient, one insufficient')
