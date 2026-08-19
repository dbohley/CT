"""Latest-frame-per-ID mailboxes, and the receive thread that fills them.

Rule 4: **the control tick never blocks on I/O.** A CAN read that waits for a frame turns
every bus hiccup into a missed deadline, and missed deadlines on a needle controller are
not a performance problem. So receive runs on its own thread, drops frames into a
bounded structure, and the tick takes whatever is there.

Two shapes, because the controller wants both:

- :class:`Mailbox` keeps only the *newest* frame per CAN ID. That is what a control loop
  actually wants from a periodic status frame — an older one is not useful.
- :class:`FrameQueue` keeps everything in arrival order, bounded, dropping oldest. Codecs
  that need every frame (and the recorder) read this.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable

#: One received frame: host monotonic arrival time, CAN id, payload.
Frame = tuple[float, int, bytes]


@dataclass
class Slot:
    """The newest frame seen for one CAN ID, and how many have been seen."""

    t: float
    data: bytes
    count: int = 1

    def age(self, now: float) -> float:
        return now - self.t


class Mailbox:
    """Newest frame per CAN ID, with staleness.

    Thread-safe: the receive thread writes, the control tick reads. The lock is held only
    for a dict assignment, so the tick never waits on anything meaningful.
    """

    def __init__(self) -> None:
        self._slots: dict[int, Slot] = {}
        self._lock = threading.Lock()

    def put(self, t: float, can_id: int, data: bytes) -> None:
        with self._lock:
            slot = self._slots.get(can_id)
            if slot is None:
                self._slots[can_id] = Slot(t=t, data=data)
            else:
                slot.t, slot.data = t, data
                slot.count += 1

    def put_many(self, frames: Iterable[Frame]) -> None:
        for t, can_id, data in frames:
            self.put(t, can_id, data)

    def get(self, can_id: int) -> Slot | None:
        with self._lock:
            return self._slots.get(can_id)

    def age(self, can_id: int, now: float) -> float | None:
        """Seconds since the newest frame for this ID, or ``None`` if never seen."""
        slot = self.get(can_id)
        return None if slot is None else slot.age(now)

    def is_stale(self, can_id: int, now: float, budget_s: float) -> bool:
        """True if nothing has arrived within ``budget_s``.

        A never-seen ID counts as stale: at startup that is exactly right, and it means
        callers do not have to special-case the first tick.
        """
        age = self.age(can_id, now)
        return age is None or age > budget_s

    def snapshot(self) -> dict[int, Slot]:
        with self._lock:
            return dict(self._slots)

    def ids(self) -> list[int]:
        with self._lock:
            return sorted(self._slots)

    def clear(self) -> None:
        with self._lock:
            self._slots.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._slots)


class FrameQueue:
    """Bounded FIFO of frames in arrival order, dropping oldest when full.

    Dropping the *oldest* is the right policy here: if the consumer has fallen behind,
    recent frames describe the world and old ones do not. The drop count is surfaced so a
    silently-overflowing queue shows up in the run summary instead of just producing
    strange behaviour.
    """

    def __init__(self, maxlen: int = 4096) -> None:
        if maxlen <= 0:
            raise ValueError("maxlen must be positive")
        self._q: deque[Frame] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self.received = 0
        self.dropped = 0

    def put(self, t: float, can_id: int, data: bytes) -> None:
        with self._lock:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
            self._q.append((t, can_id, data))
            self.received += 1

    def drain(self) -> list[Frame]:
        """Take everything queued and leave it empty. The tick's single read."""
        with self._lock:
            out = list(self._q)
            self._q.clear()
            return out

    @property
    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {"received": self.received, "dropped": self.dropped, "queued": len(self._q)}

    def __len__(self) -> int:
        with self._lock:
            return len(self._q)


class ReceiveThread:
    """Runs a blocking read in the background and feeds a queue and a mailbox.

    Owned by the real bus. The loopback bus has no need for it — there is no blocking
    read to get off the tick — which is itself a small argument that the abstraction is
    in the right place.
    """

    def __init__(self, read_one, queue: FrameQueue, mailbox: Mailbox, name: str = "can-rx") -> None:
        self._read_one = read_one
        """Callable returning ``(t, can_id, data)`` or ``None`` on timeout."""
        self._queue = queue
        self._mailbox = mailbox
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self.errors = 0
        self.last_error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._read_one()
            except Exception as exc:  # noqa: BLE001 - a dead RX thread must not kill the process
                # The control loop's watchdog notices the staleness and faults cleanly.
                # Raising here would take out the thread and leave the axes commanded.
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                continue
            if frame is None:
                continue
            t, can_id, data = frame
            self._queue.put(t, can_id, data)
            self._mailbox.put(t, can_id, data)
