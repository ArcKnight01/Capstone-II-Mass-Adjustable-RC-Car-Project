"""
ServoDriver_Util.py - PCA9685 servo driver utilities for the RC car.

Provides low-level helpers for PWM/angle conversions and a ``ServoDriver``
class that wraps ``adafruit_servokit.ServoKit`` to output steering and
throttle commands via an Adafruit PCA9685 I2C PWM driver board.

Hardware assumptions
--------------------
- Steering servo on PCA9685 channel 0 (STEERING_CHANNEL).
- ESC on PCA9685 channel 1 (ESC_CHANNEL).
- Standard RC pulse range 1000–2000 µs at 50 Hz.
- Servo actuation range 0–180°, neutral at 90°.
- Steering angle convention: negative = left, positive = right
  (matches the ``negative-left`` sign convention used throughout the project).
- Steering physical limits: ±60° of servo travel.

Servo-to-road-wheel transfer function
--------------------------------------
The true road-wheel angle as a function of servo angle has **not yet been
measured**.  ``steering_deg_to_servo_deg()`` and ``road_wheel_rad_to_servo_deg()``
use a 1:1 placeholder until the linkage geometry is characterised.

TODO: Replace the placeholder with a polynomial or lookup-table fit from a
      measured servo-angle vs road-wheel-angle dataset.

Usage
-----
    driver = ServoDriver()
    driver.set_steering(-15.0)   # 15° left
    driver.set_throttle(0.3)     # 30 % forward throttle
    driver.stop()
    driver.close()

    # or as a context manager
    with ServoDriver() as driver:
        driver.set_steering(10.0)
        driver.set_throttle(0.5)
"""

from __future__ import annotations

import math
import os

try:
    robotSupported = os.uname().nodename in ('terminatorpi', 'robotpi', 'carpi')
except Exception:
    import platform
    robotSupported = platform.uname().node in ('terminatorpi', 'robotpi', 'carpi')

if robotSupported:
    import board
    import busio
    from adafruit_servokit import ServoKit  # type: ignore[import-untyped]

# ── Channel assignments ──────────────────────────────────────────────────────
STEERING_CHANNEL: int = 0
ESC_CHANNEL: int = 1

# ── Servo PWM parameters ─────────────────────────────────────────────────────
SERVO_PWM_FREQUENCY_HZ: int = 50
SERVO_PWM_MIN_US: int = 1000
SERVO_PWM_MAX_US: int = 2000
SERVO_ACTUATION_RANGE_DEG: int = 180
SERVO_NEUTRAL_DEG: float = 90.0

# ── Physical steering limits ─────────────────────────────────────────────────
STEERING_MIN_DEG: float = -60.0
STEERING_MAX_DEG: float = 60.0


# ── Utility functions ────────────────────────────────────────────────────────

def map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """
    Linearly map ``value`` from one numeric range to another.

    Parameters
    ----------
    value : float
        Input value to remap.
    in_min : float
        Lower bound of the input range.
    in_max : float
        Upper bound of the input range.
    out_min : float
        Lower bound of the output range.
    out_max : float
        Upper bound of the output range.

    Returns
    -------
    float
        Value scaled into ``[out_min, out_max]``.  The result is *not* clamped,
        so out-of-range inputs produce out-of-range outputs.
    """
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)


def pulse_to_angle(
    pulse_us: float,
    pulse_min: int = SERVO_PWM_MIN_US,
    pulse_max: int = SERVO_PWM_MAX_US,
    actuation_range: int = SERVO_ACTUATION_RANGE_DEG,
) -> float:
    """
    Convert an RC PWM pulse width to a servo angle.

    Parameters
    ----------
    pulse_us : float
        Pulse width in microseconds.
    pulse_min : int, optional
        Pulse width corresponding to 0°. Defaults to 1000 µs.
    pulse_max : int, optional
        Pulse width corresponding to ``actuation_range``°. Defaults to 2000 µs.
    actuation_range : int, optional
        Total angular travel of the servo in degrees. Defaults to 180°.

    Returns
    -------
    float
        Servo angle in degrees, within ``[0, actuation_range]``.
    """
    angle = map_range(pulse_us, pulse_min, pulse_max, 0.0, float(actuation_range))
    return max(0.0, min(float(actuation_range), angle))


def angle_to_pulse(
    angle_deg: float,
    pulse_min: int = SERVO_PWM_MIN_US,
    pulse_max: int = SERVO_PWM_MAX_US,
    actuation_range: int = SERVO_ACTUATION_RANGE_DEG,
) -> float:
    """
    Convert a servo angle to an RC PWM pulse width.

    Parameters
    ----------
    angle_deg : float
        Servo angle in degrees, within ``[0, actuation_range]``.
    pulse_min : int, optional
        Pulse width corresponding to 0°. Defaults to 1000 µs.
    pulse_max : int, optional
        Pulse width corresponding to ``actuation_range``°. Defaults to 2000 µs.
    actuation_range : int, optional
        Total angular travel of the servo in degrees. Defaults to 180°.

    Returns
    -------
    float
        Pulse width in microseconds.
    """
    return map_range(angle_deg, 0.0, float(actuation_range), pulse_min, pulse_max)


def steering_deg_to_servo_deg(
    steering_angle_deg: float,
    steering_direction: int = 1,
) -> float:
    """
    Convert a steering command angle to a physical servo angle.

    The steering angle follows the project sign convention (negative = left,
    positive = right).  The servo neutral position is ``SERVO_NEUTRAL_DEG``
    (90°), and steering travel is offset from there.

    .. note::
        The current implementation is a **1:1 placeholder**.  The actual
        road-wheel angle is a nonlinear function of servo angle due to
        Ackermann / steering-arm geometry.

        TODO: Replace with a polynomial or lookup-table fit from measured
        servo-angle vs road-wheel-angle calibration data.

    Parameters
    ----------
    steering_angle_deg : float
        Desired steering angle in degrees.  Negative = left, positive = right.
        Values outside ``[STEERING_MIN_DEG, STEERING_MAX_DEG]`` are clamped.
    steering_direction : int, optional
        +1 if increasing servo angle turns the wheels right (default), -1 if
        increasing servo angle turns left.  Flip this if the servo is mounted
        in reverse.

    Returns
    -------
    float
        Servo angle in degrees, within ``[0, SERVO_ACTUATION_RANGE_DEG]``.
    """
    clamped = max(STEERING_MIN_DEG, min(STEERING_MAX_DEG, steering_angle_deg))
    servo_deg = SERVO_NEUTRAL_DEG + steering_direction * clamped
    return max(0.0, min(float(SERVO_ACTUATION_RANGE_DEG), servo_deg))


def road_wheel_rad_to_servo_deg(
    road_wheel_rad: float,
    steering_direction: int = 1,
) -> float:
    """
    Convert a desired road-wheel angle (radians) to a servo angle (degrees).

    Converts ``road_wheel_rad`` to degrees and delegates to
    :func:`steering_deg_to_servo_deg`.

    .. note::
        Uses the same 1:1 placeholder as :func:`steering_deg_to_servo_deg`.

        TODO: Replace with a calibrated inverse transfer function once the
        servo-angle vs road-wheel-angle mapping is measured.

    Parameters
    ----------
    road_wheel_rad : float
        Desired road-wheel steering angle in radians.
        Negative = left, positive = right.
    steering_direction : int, optional
        +1 if increasing servo angle turns the wheels right (default), -1
        otherwise.

    Returns
    -------
    float
        Servo angle in degrees, within ``[0, SERVO_ACTUATION_RANGE_DEG]``.
    """
    return steering_deg_to_servo_deg(
        math.degrees(road_wheel_rad),
        steering_direction=steering_direction,
    )


# ── ServoDriver class ────────────────────────────────────────────────────────

class ServoDriver:
    """
    High-level driver for steering servo and ESC via an Adafruit PCA9685 board.

    Wraps ``adafruit_servokit.ServoKit`` to provide steering and throttle
    commands with built-in angle clamping, sign-convention handling, and
    safe shutdown.

    On non-robot hardware the driver operates in **stub mode**: all commands
    are silently accepted and stored but no I2C traffic is generated.  This
    lets the rest of the code import and instantiate ``ServoDriver`` on a
    development machine without errors.

    Parameters
    ----------
    steering_channel : int, optional
        PCA9685 channel for the steering servo. Defaults to ``STEERING_CHANNEL`` (0).
    esc_channel : int, optional
        PCA9685 channel for the ESC. Defaults to ``ESC_CHANNEL`` (1).
    steering_direction : int, optional
        +1 if increasing servo angle turns the wheels right (default).  Set to
        -1 if the servo is mechanically reversed.
    num_channels : int, optional
        Number of channels on the PCA9685 board (8 or 16). Defaults to 16.

    Attributes
    ----------
    steering_angle_deg : float
        Most recently commanded steering angle (degrees, negative = left).
    throttle : float
        Most recently commanded throttle value (-1.0 to 1.0).
    is_hardware : bool
        ``True`` when running on supported robot hardware.

    Examples
    --------
    >>> driver = ServoDriver()
    >>> driver.set_steering(-15.0)
    >>> driver.set_throttle(0.3)
    >>> driver.stop()
    >>> driver.close()
    """

    def __init__(
        self,
        steering_channel: int = STEERING_CHANNEL,
        esc_channel: int = ESC_CHANNEL,
        steering_direction: int = 1,
        num_channels: int = 16,
    ) -> None:
        self._steering_channel = steering_channel
        self._esc_channel = esc_channel
        self._steering_direction = steering_direction
        self.steering_angle_deg: float = 0.0
        self.throttle: float = 0.0
        self.is_hardware: bool = robotSupported

        self._kit = None
        if robotSupported:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._kit = ServoKit(channels=num_channels, i2c=i2c)
            self._kit.servo[self._steering_channel].set_pulse_width_range(
                SERVO_PWM_MIN_US, SERVO_PWM_MAX_US
            )
            self._kit.servo[self._steering_channel].actuation_range = SERVO_ACTUATION_RANGE_DEG
            self._kit.continuous_servo[self._esc_channel].set_pulse_width_range(
                SERVO_PWM_MIN_US, SERVO_PWM_MAX_US
            )
            self.center_steering()
            self.coast()

    def set_steering(self, angle_deg: float) -> None:
        """
        Command the steering servo to the given angle.

        The angle is clamped to ``[STEERING_MIN_DEG, STEERING_MAX_DEG]`` before
        conversion so the linkage can never be driven out of range.

        Parameters
        ----------
        angle_deg : float
            Desired steering angle in degrees.  Negative = left, positive = right.

        Returns
        -------
        None
        """
        servo_deg = steering_deg_to_servo_deg(angle_deg, self._steering_direction)
        self.steering_angle_deg = max(STEERING_MIN_DEG, min(STEERING_MAX_DEG, angle_deg))
        if self._kit is not None:
            self._kit.servo[self._steering_channel].angle = servo_deg

    def set_steering_rad(self, angle_rad: float) -> None:
        """
        Command the steering servo using a road-wheel angle in radians.

        Converts ``angle_rad`` to degrees and delegates to :meth:`set_steering`.

        Parameters
        ----------
        angle_rad : float
            Desired road-wheel angle in radians.  Negative = left, positive = right.

        Returns
        -------
        None
        """
        self.set_steering(math.degrees(angle_rad))

    def set_throttle(self, throttle: float) -> None:
        """
        Command the ESC to the given throttle level.

        Parameters
        ----------
        throttle : float
            Throttle value in ``[-1.0, 1.0]``.  Positive = forward,
            negative = reverse, 0 = coast.  Values outside the range are
            clamped.

        Returns
        -------
        None
        """
        clamped = max(-1.0, min(1.0, throttle))
        self.throttle = clamped
        if self._kit is not None:
            self._kit.continuous_servo[self._esc_channel].throttle = clamped

    def center_steering(self) -> None:
        """
        Return the steering servo to the neutral (straight-ahead) position.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.set_steering(0.0)

    def coast(self) -> None:
        """
        Set the ESC to zero throttle (coast, not active brake).

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.set_throttle(0.0)

    def stop(self) -> None:
        """
        Center the steering and cut throttle to zero.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.center_steering()
        self.coast()

    def close(self) -> None:
        """
        Stop outputs and release hardware resources.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.stop()
        if self._kit is not None and hasattr(self._kit, '_pca'):
            try:
                self._kit._pca.deinit()
            except Exception:
                pass

    def __enter__(self) -> ServoDriver:
        return self

    def __exit__(self, _exc_type, _exc_val, _exc_tb) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"ServoDriver(steering={self.steering_angle_deg:.1f}°, "
            f"throttle={self.throttle:.2f}, "
            f"hardware={self.is_hardware})"
        )


if __name__ == "__main__":
    import time

    print("ServoDriver test")
    print(f"  Running on robot hardware: {robotSupported}")

    print(f"  map_range(1500, 1000, 2000, 0, 180) = {map_range(1500, 1000, 2000, 0, 180)}")
    print(f"  pulse_to_angle(1500) = {pulse_to_angle(1500)}")
    print(f"  angle_to_pulse(90)   = {angle_to_pulse(90)}")
    print(f"  steering_deg_to_servo_deg(0)   = {steering_deg_to_servo_deg(0)}")
    print(f"  steering_deg_to_servo_deg(30)  = {steering_deg_to_servo_deg(30)}")
    print(f"  steering_deg_to_servo_deg(-30) = {steering_deg_to_servo_deg(-30)}")

    with ServoDriver() as driver:
        print(driver)
        for angle in [0.0, 30.0, -30.0, 60.0, -60.0, 0.0]:
            driver.set_steering(angle)
            print(f"  set_steering({angle:+.0f}°) → servo {steering_deg_to_servo_deg(angle):.1f}°")
            time.sleep(0.5)
        driver.set_throttle(0.2)
        print(f"  set_throttle(0.2) → {driver.throttle}")
        time.sleep(0.5)

    print("Done.")
