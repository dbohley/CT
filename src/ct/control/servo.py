"""The needle servo: a lead compensator, and the ``tau_cl(omega_r)`` it implies.

Two jobs, and the second is the one that matters to the rest of the system.

**Tracking.** During INSERT the needle holds a constant standoff while the skin recedes,
which is a reference-tracking problem against a moving target. :class:`LeadServo` is the
discrete compensator that does it.

**Supplying ``tau_cl``.** The horizon is ``h = tau_s + tau_c + tau_cl(omega_r) + T_ins``,
and ``CLAUDE.md`` is emphatic about what the third term is: not the raw actuator delay
``tau_a``, but the *residual closed-loop tracking lag* — how far behind the reference the
compensated loop actually runs at the frequency it is being asked to follow. That is a
property of the closed loop, so it is computed here::

    L(s) = C(s) G(s)
    T(s) = L(s) / (1 + L(s))
    tau_cl(omega) = -angle(T(j*omega)) / omega

A phase lag of ``phi`` radians at ``omega`` *is* a time lag of ``phi/omega`` seconds, so
this is the delay the loop genuinely exhibits at that breathing rate — and it changes as
the rate does, which is precisely why it cannot be a configured constant. Feeding a fixed
number in its place is the error the settled decision exists to prevent.

The plant here is a second-order approximation of the needle axis. Until someone runs a
step response on the real hardware it is a placeholder, and :mod:`ct.unknowns` says so.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel


def plant_response(plant: PlantModel, omega: np.ndarray | float) -> np.ndarray:
    """``G(j*omega)`` for ``K*wn^2 / (s * (s^2 + 2*zeta*wn*s + wn^2))``.

    Note the integrator. A position axis is **type 1**: the motor produces velocity and
    position is its integral. Modelling it without one — as a plain second-order lag —
    gives a loop with finite DC gain, and two things then go wrong at breathing
    frequencies. The closed loop tracks only a fraction of its reference (``|T|`` was 0.55
    with the placeholder gains, so the needle would follow barely half the skin's motion),
    and its phase comes out *positive*, making ``tau_cl`` clamp to zero and quietly drop a
    term out of the horizon. Both were observed before the integrator was added.

    ``wn`` and ``zeta`` then describe the axis's mechanical resonance, which is what a
    step-response identification on the real needle drive will actually measure.
    """
    s = 1j * np.asarray(omega, dtype=float)
    wn, zeta = plant.wn, plant.zeta
    return plant.K * wn**2 / (s * (s**2 + 2 * zeta * wn * s + wn**2))


def lead_response(lead: LeadCompensator, omega: np.ndarray | float) -> np.ndarray:
    """``C(j*omega)`` for ``gain*(s + zero)/(s + pole)``."""
    s = 1j * np.asarray(omega, dtype=float)
    return lead.gain * (s + lead.zero) / (s + lead.pole)


def closed_loop_response(config: AxisServoConfig, omega: np.ndarray | float) -> np.ndarray:
    """``T(j*omega) = L/(1+L)`` for the compensated axis."""
    L = lead_response(config.lead, omega) * plant_response(config.plant, omega)
    return L / (1.0 + L)


def residual_lag(config: AxisServoConfig, omega_r: float) -> float:
    """``tau_cl(omega_r)`` — seconds of residual tracking lag at that breathing rate.

    Guarded at ``omega_r -> 0``: the ratio ``-angle/omega`` is finite in the limit but
    numerically hopeless near zero, so below a small threshold it is evaluated at the
    threshold instead. Breathing is never that slow, and a NaN in the horizon would
    propagate into every forecast.
    """
    omega_r = float(omega_r)
    if omega_r < 0:
        raise ValueError(f"omega_r must be non-negative, got {omega_r}")
    omega_eval = max(omega_r, 1e-3)
    phase = float(np.angle(closed_loop_response(config, omega_eval)))
    # A well-designed loop lags (negative phase). Should a design lead at this frequency,
    # report zero rather than a negative horizon term: `horizon_from_components` refuses
    # negatives, and "the servo is ahead" is not a reason to forecast into the past.
    return max(0.0, -phase / omega_eval)


def bandwidth(config: AxisServoConfig, omega_max: float = 1e4, n: int = 4096) -> float:
    """Closed-loop -3 dB bandwidth [rad/s]. Reported in the run summary as a sanity check.

    Worth a glance: if this is not comfortably above the breathing harmonics the model is
    tracking, the servo cannot follow the reference regardless of what the forecast says.
    """
    omega = np.logspace(-2, np.log10(omega_max), n)
    mag = np.abs(closed_loop_response(config, omega))
    below = np.where(mag < mag[0] / np.sqrt(2.0))[0]
    return float(omega[below[0]]) if below.size else float(omega[-1])


class LeadServo:
    """Discrete lead compensator, Tustin-discretised.

    ``C(s) = gain*(s + zero)/(s + pole)`` with ``s = (2/Ts)*(1 - z^-1)/(1 + z^-1)`` gives

        y[n] = (b0*u[n] + b1*u[n-1] - a1*y[n-1]) / a0

    Tustin rather than a zero-order-hold equivalent because it preserves the frequency
    response near the breathing band, which is the band that matters here — the warping is
    at the Nyquist end, far above anything the chest is doing.
    """

    def __init__(self, config: AxisServoConfig, Ts: float) -> None:
        if Ts <= 0:
            raise ValueError("Ts must be positive")
        self.config = config
        self.Ts = float(Ts)
        lead = config.lead

        a = 2.0 / self.Ts
        self._b0 = lead.gain * (a + lead.zero)
        self._b1 = lead.gain * (lead.zero - a)
        self._a0 = a + lead.pole
        self._a1 = lead.pole - a

        self._u_prev = 0.0
        self._y_prev = 0.0
        self.saturated = 0
        self.rate_limited = 0

    def reset(self, u0: float = 0.0, y0: float = 0.0) -> None:
        """Clear the filter memory. Called on entering a state that uses the servo."""
        self._u_prev = float(u0)
        self._y_prev = float(y0)

    def update(
        self, error: float, limit: float | None = None, rate_limit_mm_s: float | None = None
    ) -> float:
        """One step. ``error`` is reference minus measurement; returns the correction.

        ``limit`` bounds the correction's magnitude; ``rate_limit_mm_s`` separately bounds
        how much it may change from the *previous* call, in the correction's own units per
        second (millimetres, despite the name inherited from the velocity-limit call sites —
        see ``ct.hw.config.AxisServoConfig``). Neither is optional in spirit for a real
        control loop: a magnitude clamp alone still lets a large step error produce a full-size
        correction in one tick, which is exactly the failure mode a lead compensator's zero
        is prone to on a sudden error. Both default to the values baked into ``self.config``
        at construction, so a call site that just does ``update(error)`` gets the configured
        safety envelope automatically rather than needing to remember (or mis-remember) one.
        """
        if limit is None:
            limit = self.config.correction_limit_mm
        if rate_limit_mm_s is None:
            rate_limit_mm_s = self.config.correction_rate_limit_mm_s

        y = (self._b0 * error + self._b1 * self._u_prev - self._a1 * self._y_prev) / self._a0
        if limit is not None and abs(y) > limit:
            # Clamp the *output* but store the clamped value as the state, so the filter
            # cannot wind up a history it will never work off.
            y = float(np.clip(y, -limit, limit))
            self.saturated += 1
        if rate_limit_mm_s is not None:
            max_step = rate_limit_mm_s * self.Ts
            if abs(y - self._y_prev) > max_step:
                y = float(np.clip(y, self._y_prev - max_step, self._y_prev + max_step))
                self.rate_limited += 1
        self._u_prev = float(error)
        self._y_prev = y
        return y

    def residual_lag(self, omega_r: float) -> float:
        """``tau_cl`` at this breathing rate. Bound into the latency budget."""
        return residual_lag(self.config, omega_r)

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "Ts": self.Ts,
            "zero": self.config.lead.zero,
            "pole": self.config.lead.pole,
            "gain": self.config.lead.gain,
            "alpha": self.config.lead.alpha,
            "bandwidth_rad_s": bandwidth(self.config),
            "saturated_steps": self.saturated,
            "rate_limited_steps": self.rate_limited,
        }

    def __repr__(self) -> str:
        lead = self.config.lead
        return f"LeadServo(zero={lead.zero}, pole={lead.pole}, gain={lead.gain}, Ts={self.Ts})"
