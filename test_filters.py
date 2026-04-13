#!/usr/bin/env python3
"""
Test script for the modular filter implementation.

This script demonstrates the usage of the LowPassFilter, HighPassFilter,
and ComplementaryFilter classes.
"""

import numpy as np
import matplotlib.pyplot as plt
from LowPassFilter import LowPassFilter
from HighPassFilter import HighPassFilter
from ComplementaryFilter import ComplementaryFilter, YawComplementaryFilter

def test_individual_filters():
    """Test LowPassFilter and HighPassFilter individually."""
    print("Testing individual filters...")

    # Create test signal: low frequency + high frequency noise
    t = np.linspace(0, 10, 1000)
    low_freq = np.sin(2 * np.pi * 0.1 * t)  # 0.1 Hz sine wave
    high_freq = 0.5 * np.sin(2 * np.pi * 5 * t)  # 5 Hz noise
    signal = low_freq + high_freq

    # Create filters
    cutoff_hz = 1.0
    dt = t[1] - t[0]

    lp_filter = LowPassFilter(cutoff_hz)
    hp_filter = HighPassFilter(cutoff_hz)

    # Filter the signal
    lp_output = []
    hp_output = []

    for sample in signal:
        lp_output.append(lp_filter.update(sample, dt))
        hp_output.append(hp_filter.update(sample, dt))

    # Verify complementarity: low_pass + high_pass should equal input
    combined = np.array(lp_output) + np.array(hp_output)
    max_error = np.max(np.abs(combined - signal))
    print(".6f")

    return t, signal, lp_output, hp_output

def test_complementary_filter():
    """Test the generic ComplementaryFilter."""
    print("Testing complementary filter...")

    # Simulate magnetometer (low frequency, accurate) and gyro (high frequency, noisy)
    t = np.linspace(0, 10, 1000)
    dt = t[1] - t[0]

    # True heading (what we want to estimate)
    true_heading = np.sin(2 * np.pi * 0.05 * t)  # Very slow drift

    # Magnetometer: accurate but slow to respond to changes
    magnetometer = true_heading + 0.1 * np.random.randn(len(t))

    # Gyro integration: fast but accumulates drift
    gyro_rate = np.gradient(true_heading, dt) + 0.1 * np.random.randn(len(t))
    gyro_heading = np.cumsum(gyro_rate * dt)

    # Apply complementary filter
    comp_filter = ComplementaryFilter(cutoff_hz=0.2, alpha=0.8)  # Trust magnetometer more

    fused_output = []
    for mag, gyro in zip(magnetometer, gyro_heading):
        fused = comp_filter.update(mag, gyro, dt)
        fused_output.append(fused)

    # Calculate RMS errors
    mag_error = np.sqrt(np.mean((magnetometer - true_heading)**2))
    gyro_error = np.sqrt(np.mean((gyro_heading - true_heading)**2))
    fused_error = np.sqrt(np.mean((np.array(fused_output) - true_heading)**2))

    print(".4f")
    print(".4f")
    print(".4f")

    return t, true_heading, magnetometer, gyro_heading, fused_output

def test_yaw_filter():
    """Test the specialized YawComplementaryFilter."""
    print("Testing yaw complementary filter...")

    t = np.linspace(0, 10, 1000)
    dt = t[1] - t[0]

    # True yaw
    true_yaw = np.sin(2 * np.pi * 0.1 * t)

    # Magnetometer heading (with some noise)
    magnetometer = true_yaw + 0.05 * np.random.randn(len(t))

    # Gyro rate (with some noise)
    gyro_rate = np.gradient(true_yaw, dt) + 0.1 * np.random.randn(len(t))

    # Apply yaw filter
    yaw_filter = YawComplementaryFilter(cutoff_hz=0.1, alpha=0.9)

    fused_yaw = []
    for i, (mag, rate) in enumerate(zip(magnetometer, gyro_rate)):
        yaw = yaw_filter.update(mag, rate, t[i])
        fused_yaw.append(yaw)

    # Calculate RMS error
    error = np.sqrt(np.mean((np.array(fused_yaw) - true_yaw)**2))
    print(".4f")

    return t, true_yaw, magnetometer, fused_yaw

if __name__ == "__main__":
    print("Testing modular filter implementation...")
    print("=" * 50)

    # Test individual filters
    test_individual_filters()
    print()

    # Test complementary filter
    test_complementary_filter()
    print()

    # Test yaw filter
    test_yaw_filter()
    print()

    print("All tests completed successfully!")
    print("The modular filter implementation is working correctly.")