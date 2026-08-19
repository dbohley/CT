"""The real bus: a USB-CAN-FD RH02 Plus adapter, through python-can.

There is nothing RH02-specific in the code — python-can abstracts the adapter, and what
the RH02 needs is the right ``interface``/``channel`` pair in the config. That pairing is
an open unknown (see :mod:`ct.unknowns`), because these adapters enumerate differently
per platform: usually SocketCAN via gs_usb/candleLight on Linux, a vendor backend on
Windows. Find out with::

    python -m can.detect_available_configs
    python -m can.viewer -i socketcan -c can0

**Classic CAN, not FD, for the motor bus.** The RH02 is FD-capable but CubeMars AK motors
are classic CAN 2.0 at 1 Mbit/s. Set ``fd: false`` on any bus carrying them. ``fd: true``
is there for a genuinely FD device on the sensing side.

``python-can`` is imported inside the constructor rather than at module scope, so that
``import ct.hw`` — and therefore the whole simulated stack and its tests — works on a
machine that has never had a CAN adapter attached.
"""

from __future__ import annotations

from typing import Any

from ct.hw.bus.mailbox import Frame, FrameQueue, Mailbox, ReceiveThread
from ct.registry import register_bus


@register_bus("rh02")
class RH02Bus:
    """python-can transport with the receive side on its own thread (rule 4)."""

    def __init__(
        self,
        name: str = "rh02",
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 1_000_000,
        fd: bool = False,
        data_bitrate: int = 2_000_000,
        rx_queue: int = 4096,
        read_timeout_s: float = 0.05,
    ) -> None:
        try:
            import can  # noqa: PLC0415 - deliberately lazy; see the module docstring
        except ImportError as exc:  # pragma: no cover - depends on the host
            raise ImportError(
                "python-can is required for the 'rh02' bus backend but is not installed.\n"
                "  pip install python-can\n"
                "Simulation does not need it: use `backend: loopback`."
            ) from exc

        self.name = name
        self.interface = interface
        self.channel = channel
        self._read_timeout_s = float(read_timeout_s)

        kwargs: dict[str, Any] = {"interface": interface, "channel": channel, "bitrate": bitrate}
        if fd:
            kwargs["fd"] = True
            kwargs["data_bitrate"] = data_bitrate
        self._bus = can.Bus(**kwargs)

        self._queue = FrameQueue(maxlen=rx_queue)
        self.mailbox = Mailbox()
        self.sent = 0
        self.send_errors = 0
        self._closed = False

        self._rx = ReceiveThread(self._read_one, self._queue, self.mailbox, name=f"can-rx-{name}")
        self._rx.start()

    # -- receive ---------------------------------------------------------------

    def _read_one(self) -> Frame | None:
        """One blocking read, off the control tick. Returns ``None`` on timeout.

        python-can stamps messages with ``time.time()``. The rest of this package works
        in ``time.monotonic()`` — including the phantom logger, which has to align across
        processes — so the arrival time is taken here rather than trusted from the driver.
        The error against the true bus arrival is the read latency, which is small and,
        more usefully, is the same for every frame.
        """
        import time  # noqa: PLC0415 - paired with the lazy `can` import above

        msg = self._bus.recv(timeout=self._read_timeout_s)
        if msg is None:
            return None
        return (time.monotonic(), int(msg.arbitration_id), bytes(msg.data))

    # -- Bus protocol ----------------------------------------------------------

    def send(self, can_id: int, data: bytes, *, extended: bool = False) -> None:
        import can  # noqa: PLC0415

        if self._closed:
            raise RuntimeError(f"bus '{self.name}' is closed")
        msg = can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=extended)
        try:
            self._bus.send(msg, timeout=0.0)
            self.sent += 1
        except Exception:  # noqa: BLE001
            # A full TX buffer must not raise into the control tick. The command is lost;
            # the next tick sends a fresh one, which is more useful than a stale retry.
            # A rising count here means the bus is saturated and the loop rate is too high
            # for the frame budget.
            self.send_errors += 1

    def poll(self) -> list[Frame]:
        return self._queue.drain()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._rx.stop()
        self._bus.shutdown()

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "backend": "rh02",
            "interface": self.interface,
            "channel": self.channel,
            "sent": self.sent,
            "send_errors": self.send_errors,
            "rx_errors": self._rx.errors,
            "rx_last_error": self._rx.last_error,
            "rx_alive": self._rx.is_alive,
            **self._queue.stats,
        }

    def set_time(self, t: float) -> None:
        """No-op. Real time comes from the adapter; accepted so the loop is backend-blind."""

    def __repr__(self) -> str:
        return f"RH02Bus(interface={self.interface!r}, channel={self.channel!r})"
