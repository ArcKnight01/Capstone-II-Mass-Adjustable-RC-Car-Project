"""
Low-pass filter implementation for signal processing.

A low-pass filter attenuates high-frequency components while allowing
low-frequency components to pass through. This is useful for removing
noise and smoothing signals.
"""

import math


class LowPassFilter:
    """A simple first-order low-pass filter for smoothing data."""

    __slots__ = ("cutoff_hz", "_value", "_initialized")

    def __init__(self, cutoff_hz: float):
        """
        Initialize the low-pass filter.

        Parameters
        ----------
        cutoff_hz : float
            Cutoff frequency in Hz. Signals above this frequency will be attenuated.
        """
        cutoff_hz = float(cutoff_hz)
        if cutoff_hz <= 0.0:
            raise ValueError("cutoff_hz must be positive")

        self.cutoff_hz = cutoff_hz
        self._value = 0.0
        self._initialized = False

    def update(self, new_value: float, dt: float) -> float:
        """
        Update the filter with a new value.

        Parameters
        ----------
        new_value : float
            The new input value to filter.
        dt : float
            Time step in seconds since the last update.

        Returns
        -------
        float
            The filtered output value.
        """
        value = float(new_value)
        dt = max(0.0, float(dt))

        if not self._initialized or dt <= 0.0:
            self._value = value
            self._initialized = True
            return self._value

        # First-order RC low-pass filter
        # alpha = dt / (RC + dt) where RC = 1/(2π*cutoff_hz)
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (rc + dt)
        self._value = self._value + alpha * (value - self._value)
        return self._value

    @property
    def value(self) -> float:
        """Get the current filtered value."""
        return self._value

    def reset(self, value: float = 0.0) -> None:
        """Reset the filter to a specific value."""
        self._value = float(value)
        self._initialized = True