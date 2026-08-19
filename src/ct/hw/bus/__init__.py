"""CAN transports.

``loopback`` is simulation; ``rh02`` is the USB-CAN-FD adapter. The real backend is
imported lazily by :func:`build_bus_from_config` so that python-can stays an optional
dependency — a simulated run must not require a CAN driver to be installed.
"""

from __future__ import annotations

from typing import Any

from ct.hw.bus.loopback import LoopbackBus
from ct.hw.bus.mailbox import Frame, FrameQueue, Mailbox, ReceiveThread
from ct.hw.config import BusConfig

__all__ = [
    "LoopbackBus",
    "Mailbox",
    "FrameQueue",
    "ReceiveThread",
    "Frame",
    "build_bus_from_config",
]


def build_bus_from_config(name: str, config: BusConfig) -> Any:
    """Construct the bus a :class:`~ct.hw.config.BusConfig` describes."""
    if config.backend == "loopback":
        return LoopbackBus(name=name, rx_queue=config.rx_queue)
    if config.backend == "rh02":
        from ct.hw.bus.canfd_rh02 import RH02Bus  # noqa: PLC0415 - keeps python-can optional

        return RH02Bus(
            name=name,
            interface=config.interface,
            channel=config.channel,
            bitrate=config.bitrate,
            fd=config.fd,
            data_bitrate=config.data_bitrate,
            rx_queue=config.rx_queue,
        )
    raise KeyError(f"unknown bus backend '{config.backend}'. Known: loopback, rh02")
