"""In-process bus. Simulation's half of the ``Bus`` boundary.

Frames sent by the controller are handed to whatever devices are attached; frames those
devices produce come back on the next poll. No threads, no sockets, no timing of its own
— it is driven entirely by the clock the control loop is using, which is what keeps a
``SimClock`` run deterministic.

Devices attach through :meth:`LoopbackBus.attach`. The simulated plant is one such
device; so is the simulated phantom motor. The controller cannot tell the difference
between this and :mod:`ct.hw.bus.canfd_rh02`, which is the point — and is enforced by
running the same procedure tests across both.
"""

from __future__ import annotations

from typing import Any, Protocol

from ct.hw.bus.mailbox import Frame, FrameQueue, Mailbox
from ct.registry import register_bus


class LoopbackDevice(Protocol):
    """Something on the simulated bus that can answer.

    The simulated plant implements this. So could a recorded-trace replayer, which is how
    a captured hardware session could be played back against the controller later.
    """

    def on_frame(self, t: float, can_id: int, data: bytes, extended: bool) -> None:
        """Receive a frame the controller sent."""
        ...

    def emit(self, t: float) -> list[Frame]:
        """Frames this device wants to put on the bus at time ``t``."""
        ...


@register_bus("loopback")
class LoopbackBus:
    """A bus that goes nowhere and answers immediately."""

    def __init__(self, name: str = "loopback", rx_queue: int = 4096) -> None:
        self.name = name
        self._devices: list[LoopbackDevice] = []
        self._queue = FrameQueue(maxlen=rx_queue)
        self.mailbox = Mailbox()
        self.sent = 0
        self._closed = False
        self._t = 0.0

    def attach(self, device: LoopbackDevice) -> LoopbackDevice:
        """Put a device on the bus. Returns it, so calls can be chained or kept."""
        self._devices.append(device)
        return device

    # -- Bus protocol ----------------------------------------------------------

    def send(self, can_id: int, data: bytes, *, extended: bool = False) -> None:
        if self._closed:
            raise RuntimeError(f"bus '{self.name}' is closed")
        self.sent += 1
        for device in self._devices:
            device.on_frame(self._t, can_id, bytes(data), extended)

    def poll(self) -> list[Frame]:
        """Collect whatever the attached devices have to say, plus anything queued."""
        for device in self._devices:
            for frame in device.emit(self._t):
                self._queue.put(*frame)
                self.mailbox.put(*frame)
        return self._queue.drain()

    def close(self) -> None:
        self._closed = True

    @property
    def stats(self) -> dict[str, Any]:
        return {"backend": "loopback", "sent": self.sent, "devices": len(self._devices),
                **self._queue.stats}

    # -- simulation ------------------------------------------------------------

    def set_time(self, t: float) -> None:
        """Tell the bus what time it is.

        The loop calls this once per tick before polling. Without it the simulated
        devices would have no idea how much time had passed, since a loopback bus has no
        clock of its own — deliberately, so that time comes from exactly one place.
        """
        self._t = float(t)

    def __repr__(self) -> str:
        return f"LoopbackBus(name={self.name!r}, devices={len(self._devices)}, sent={self.sent})"
