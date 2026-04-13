"""
High-pass filter implementation for signal processing.

A high-pass filter attenuates low-frequency components while allowing
high-frequency components to pass through. This is useful for removing
drift and DC bias while preserving rapid changes.
"""

import math
from LowPassFilter import LowPassFilter


class HighPassFilter:
    """
    A simple high-pass filter that attenuates low frequencies and passes high frequencies.

    This is implemented as the complement of a low-pass filter: output = input - low_pass_output
    This ensures perfect complementarity when used with a LowPassFilter of the same cutoff.
    """

    __slots__ = ("cutoff_hz", "_low_pass")

    def __init__(self, cutoff_hz: float):
        """
        Initialize the high-pass filter.

        Parameters
        ----------
        cutoff_hz : float
            Cutoff frequency in Hz. Signals below this frequency will be attenuated.
        """
        cutoff_hz = float(cutoff_hz)
        if cutoff_hz <= 0.0:
            raise ValueError("cutoff_hz must be positive")

        self.cutoff_hz = cutoff_hz
        self._low_pass = LowPassFilter(cutoff_hz)

    def update(self, new_value: float, dt: float) -> float:
        """
        Update the high-pass filter with a new value.

        Parameters
        ----------
        new_value : float
            The new input value to filter.
        dt : float
            Time step in seconds since the last update.

        Returns
        -------
        float
            The high-frequency component (input minus low-frequency component).
        """
        value = float(new_value)
        low_pass_output = self._low_pass.update(value, dt)
        return value - low_pass_output

    @property
    def value(self) -> float:
        """Get the current high-pass filtered value."""
        return self._low_pass.value

    def reset(self, value: float = 0.0) -> None:
        """Reset the filter to a specific value."""
        self._low_pass.reset(float(value))