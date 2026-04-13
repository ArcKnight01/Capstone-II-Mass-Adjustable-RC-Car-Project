"""
Generic complementary filter for sensor fusion.

A complementary filter combines the strengths of two sensors by using
high-pass and low-pass filtering to merge their outputs. Typically,
one sensor provides accurate low-frequency information while the other
provides responsive high-frequency information.
"""

from typing import Optional
from LowPassFilter import LowPassFilter
from HighPassFilter import HighPassFilter

# https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-filtering

class ComplementaryFilter:
    """
    A generic complementary filter that combines two signals.

    This filter uses low-pass filtering on one signal (for low-frequency accuracy)
    and high-pass filtering on another signal (for high-frequency responsiveness),
    then combines them additively.

    Common use cases:
    - Magnetometer (low-pass) + Gyro integration (high-pass) for yaw estimation
    - GPS position (low-pass) + Odometry (high-pass) for position estimation
    - Accelerometer (high-pass) + Position integration (low-pass) for velocity
    """

    def __init__(self, cutoff_hz: float, alpha: Optional[float] = None):
        """
        Initialize the complementary filter.

        Parameters
        ----------
        cutoff_hz : float
            Cutoff frequency in Hz for both low-pass and high-pass filters.
        alpha : float, optional
            Weighting factor for the low-frequency signal (0.0 to 1.0).
            If None, defaults to 0.5 for equal weighting.
            alpha=1.0 means only low-frequency signal is used.
            alpha=0.0 means only high-frequency signal is used.
        """
        self.cutoff_hz = float(cutoff_hz)
        if alpha is None:
            self.alpha = 0.5  # Equal weighting
        else:
            self.alpha = max(0.0, min(1.0, float(alpha)))

        self._low_pass = LowPassFilter(cutoff_hz)
        self._high_pass = HighPassFilter(cutoff_hz)
        self._initialized = False

    def update(self, low_freq_signal: float, high_freq_signal: float, dt: float) -> float:
        """
        Update the complementary filter with new signal values.

        Parameters
        ----------
        low_freq_signal : float
            Signal that should be trusted for low frequencies (e.g., magnetometer heading).
        high_freq_signal : float
            Signal that should be trusted for high frequencies (e.g., gyro rate integration).
        dt : float
            Time step in seconds since the last update.

        Returns
        -------
        float
            The fused output combining both signals.
        """
        # Apply low-pass to the low-frequency trusted signal
        low_freq_filtered = self._low_pass.update(low_freq_signal, dt)

        # Apply high-pass to the high-frequency trusted signal
        high_freq_filtered = self._high_pass.update(high_freq_signal, dt)

        # Combine the filtered signals
        return self.alpha * low_freq_filtered + (1.0 - self.alpha) * high_freq_filtered

    def update_with_weights(self, signal1: float, signal2: float, dt: float,
                          weight1: float, weight2: float) -> float:
        """
        Update the filter with custom weights for each signal.

        Parameters
        ----------
        signal1 : float
            First signal value.
        signal2 : float
            Second signal value.
        dt : float
            Time step in seconds.
        weight1 : float
            Weight for signal1 in the low-pass filter (0.0 to 1.0).
        weight2 : float
            Weight for signal2 in the high-pass filter (0.0 to 1.0).

        Returns
        -------
        float
            The fused output.
        """
        # Apply filters
        filtered1 = self._low_pass.update(signal1, dt)
        filtered2 = self._high_pass.update(signal2, dt)

        # Combine with custom weights
        return weight1 * filtered1 + weight2 * filtered2

    @property
    def value(self) -> float:
        """Get the current filtered value."""
        return self.alpha * self._low_pass.value + (1.0 - self.alpha) * self._high_pass.value

    def reset(self, value: float = 0.0) -> None:
        """Reset both internal filters to a specific value."""
        self._low_pass.reset(value)
        self._high_pass.reset(value)
        self._initialized = True

    def set_alpha(self, alpha: float) -> None:
        """
        Change the weighting factor between low and high frequency signals.

        Parameters
        ----------
        alpha : float
            New weighting factor (0.0 to 1.0).
        """
        self.alpha = max(0.0, min(1.0, float(alpha)))


class YawComplementaryFilter:
    """
    Specialized complementary filter for yaw angle estimation.

    Combines magnetometer heading (low-frequency, absolute reference)
    with integrated gyroscope data (high-frequency, responsive).
    """

    def __init__(self, cutoff_hz: float = 0.1, alpha: float = 0.95):
        """
        Initialize the yaw complementary filter.

        Parameters
        ----------
        cutoff_hz : float
            Cutoff frequency in Hz. Default 0.1 Hz (10 second time constant).
        alpha : float
            Weight for magnetometer (low-frequency). Default 0.95 (trust magnetometer more).
        """
        self.filter = ComplementaryFilter(cutoff_hz, alpha)
        self.integrated_gyro = 0.0
        self.last_time = None

    def update(self, magnetometer_heading_rad: float, gyro_rate_radps: float, timestamp: float) -> float:
        """
        Update the yaw filter with new sensor data.

        Parameters
        ----------
        magnetometer_heading_rad : float
            Magnetometer heading in radians (after hard/soft iron calibration).
        gyro_rate_radps : float
            Gyroscope z-axis rate in radians per second.
        timestamp : float
            Current timestamp in seconds.

        Returns
        -------
        float
            Fused yaw angle in radians.
        """
        # Integrate gyro rate to get heading estimate
        if self.last_time is not None:
            dt = timestamp - self.last_time
            self.integrated_gyro += gyro_rate_radps * dt

        self.last_time = timestamp

        # Apply complementary filter
        return self.filter.update(magnetometer_heading_rad, self.integrated_gyro, dt if self.last_time else 0.01)

    def reset(self, initial_yaw: float = 0.0) -> None:
        """Reset the filter with an initial yaw angle."""
        self.integrated_gyro = initial_yaw
        self.filter.reset(initial_yaw)
        self.last_time = None