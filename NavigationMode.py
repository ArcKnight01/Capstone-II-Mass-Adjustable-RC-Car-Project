"""
NavigationMode.py - Selectable navigation pipeline modes for the RC car Controller.

Changing the mode at construction time lets you quickly compare:
  - pure IMU dead reckoning (Odometry baseline)
  - EKF fusing IMU only (indoor-safe, GPS-free)
  - EKF fusing IMU + GPS (outdoor, full sensor fusion)

All modes always run Odometry so its data appears in every CSV log for comparison.
EKF columns are populated when an EKF mode is active and left empty otherwise.
"""


class NavigationMode:
    """
    String constants for the Controller ``navigation_mode`` parameter.

    Attributes
    ----------
    ODOMETRY_ONLY : str
        Pure IMU dead reckoning via Odometry.  No EKF is created or run.
        Use this as a clean baseline to see raw IMU drift without any filter.

    EKF_IMU_ONLY : str
        EKF that fuses IMU data only — no GPS updates.  Safe for indoor
        testing where GPS is unavailable or unreliable.  The EKF initialises
        immediately from the BNO055 orientation without waiting for a GPS fix.

        In this mode the EKF:
          - predicts using body-frame longitudinal acceleration + steering angle
          - updates roll, pitch, yaw from BNO055 NDOF orientation (100 Hz, high trust)
          - updates yaw rate from gyro z
          - updates lateral acceleration from the tire-force model
          - estimates accelerometer and gyro biases online
          - position drifts slowly (no absolute reference) — expected behaviour

        Original intent: trust the BNO055 NDOF orientation completely and use
        the dynamics model + lateral accelerometer to constrain velocity better
        than raw double-integration can achieve.

    EKF_GPS_IMU : str
        Full EKF with GPS position + velocity + IMU orientation/rate/accel fusion.
        The EKF waits for the first valid GPS fix to set the local ENU origin and
        initialise position, then runs all updates each loop.

        GPS position accuracy is ~2.5 m (very large for an RC car), so its R matrix
        entry is large and it gets low weight.  GPS velocity accuracy is much better
        (~0.05 m/s) and carries more useful information at speed.

        Use outdoors once the IMU-only baseline is characterised.
    """

    ODOMETRY_ONLY = "odometry_only"
    EKF_IMU_ONLY  = "ekf_imu_only"
    EKF_GPS_IMU   = "ekf_gps_imu"

    ALL = (ODOMETRY_ONLY, EKF_IMU_ONLY, EKF_GPS_IMU)

    DESCRIPTIONS = {
        ODOMETRY_ONLY: "IMU dead reckoning only — no EKF.  Baseline for comparison.",
        EKF_IMU_ONLY:  "EKF with IMU only, no GPS.  Indoor-safe; initialises from BNO055.",
        EKF_GPS_IMU:   "Full EKF with GPS + IMU sensor fusion.  Outdoor use.",
    }

    @classmethod
    def is_valid(cls, mode: str) -> bool:
        """Return True when *mode* is a recognised navigation mode."""
        return mode in cls.ALL

    @classmethod
    def uses_ekf(cls, mode: str) -> bool:
        """Return True when *mode* requires an EKF instance."""
        return mode in (cls.EKF_IMU_ONLY, cls.EKF_GPS_IMU)

    @classmethod
    def uses_gps(cls, mode: str) -> bool:
        """Return True when *mode* feeds GPS measurements into the EKF."""
        return mode == cls.EKF_GPS_IMU
