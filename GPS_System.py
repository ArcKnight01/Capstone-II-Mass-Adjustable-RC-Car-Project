"""
GPS_System.py - GPS daemon client wrapper.

This module wraps gpsd communication to cache the latest GPS fix and expose it
in a controller-friendly dictionary. It supports both one-off reads and a
background reader thread.

Usage:
    python GPS_System.py
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
from threading import Lock, Thread
from typing import Any
import time

try:
    from gps import WATCH_ENABLE, WATCH_NEWSTYLE, gps
    
except ImportError:  # pragma: no cover - depends on robot environment
    WATCH_ENABLE = 0
    WATCH_NEWSTYLE = 0
    gps = None

from GPS_Util import (
    fix_mode_label,
    gpsd_report_get,
    gpsd_report_to_dict,
    parse_gpsd_time,
    safe_count_used_satellites,
    safe_float,
)


@dataclass(slots=True)
class GPSFix:
    """
    Cached GPS state derived from gpsd TPV/SKY reports.
    """

    timestamp: str | None = None
    timestamp_datetime: datetime | None = None
    mode: int = 0
    mode_name: str = "unknown"
    latitude: float | None = None
    longitude: float | None = None
    altitude_hae_m: float | None = None
    altitude_msl_m: float | None = None
    speed_m_s: float | None = None
    track_deg: float | None = None
    climb_m_s: float | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    epx_m: float | None = None
    epy_m: float | None = None
    eps_m_s: float | None = None
    satellites_used: int | None = None
    satellites_visible: int | None = None
    device: str | None = None
    status: int | None = None

    @property
    def has_2d_fix(self) -> bool:
        return self.mode >= 2 and self.latitude is not None and self.longitude is not None

    @property
    def has_3d_fix(self) -> bool:
        return self.mode >= 3 and self.has_2d_fix


class GPS_System:
    """
    Wrapper around gpsd that keeps the latest fix cached for controller use.

    The class can be driven in two ways:
    1. Call :meth:`update_once` from your own loop.
    2. Call :meth:`start` to run a background reader thread and then use
       :meth:`get_data` from the controller.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: str = "2947",
        verbose: bool = False,
        enabled: bool = True,
        reconnect_delay_sec: float = 1.0,
    ) -> None:
        self.__host = host
        self.__port = port
        self.__verbose = verbose
        self.__enabled = enabled
        self.__reconnect_delay_sec = reconnect_delay_sec

        self.__session: Any | None = None
        self.__lock = Lock()
        self.__worker: Thread | None = None
        self.__running = False
        self.__connected = False
        self.__last_error: str | None = None
        self.__last_fix = GPSFix()

    def connect(self) -> None:
        """
        Open a gpsd client session and enable streaming.
        """
        if not self.__enabled:
            return
        if gps is None:
            raise RuntimeError(
                "The python-gps package is not installed. Install the gpsd Python client on the robot."
            )

        if self.__session is not None:
            return

        self.__session = gps(host=self.__host, port=self.__port, mode=WATCH_ENABLE | WATCH_NEWSTYLE)
        self.__connected = False
        self.__last_error = None
        if self.__verbose:
            print(f"Connected to gpsd at {self.__host}:{self.__port}")

    def close(self) -> None:
        """
        Close the active gpsd session.
        """
        session = self.__session
        self.__session = None
        self.__connected = False
        if session is None:
            return
        try:
            session.close()
        except Exception:
            pass

    def start(self) -> None:
        """
        Start a background thread that continuously reads gpsd reports.
        """
        if not self.__enabled or self.__running:
            return

        self.__running = True
        self.__worker = Thread(target=self.__reader_loop, name="gpsd-reader", daemon=True)
        self.__worker.start()

    def stop(self) -> None:
        """
        Stop the background reader thread and close the gpsd connection.
        """
        self.__running = False
        # Closing first helps unblock a reader thread waiting on gpsd I/O.
        self.close()
        worker = self.__worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        self.__worker = None

    def update_once(self) -> GPSFix:
        """
        Read a single gpsd message and update the cached fix.
        """
        if not self.__enabled:
            return self.get_fix()

        if self.__session is None:
            self.connect()

        assert self.__session is not None
        read_result = self.__session.read()
        if read_result != 0:
            self.__connected = False
            raise RuntimeError("gpsd terminated the session or no more data is available.")

        report = getattr(self.__session, "data", None)
        if report is not None:
            self.__connected = True
            self.__last_error = None
            self.__handle_report(report)
        return self.get_fix()

    def get_fix(self) -> GPSFix:
        """
        Return the most recent GPS fix snapshot.
        """
        with self.__lock:
            return GPSFix(**asdict(self.__last_fix))

    def get_data(self) -> dict[str, object]:
        """
        Return cached GPS data in a controller-friendly dictionary.
        """
        fix = self.get_fix()
        return {
            "timestamp": fix.timestamp,
            "timestamp_datetime": fix.timestamp_datetime,
            "mode": fix.mode,
            "mode_name": fix.mode_name,
            "has_2d_fix": fix.has_2d_fix,
            "has_3d_fix": fix.has_3d_fix,
            "latitude": fix.latitude,
            "longitude": fix.longitude,
            "altitude_hae_m": fix.altitude_hae_m,
            "altitude_msl_m": fix.altitude_msl_m,
            "speed_m_s": fix.speed_m_s,
            "track_deg": fix.track_deg,
            "climb_m_s": fix.climb_m_s,
            "eph_m": fix.eph_m,
            "epv_m": fix.epv_m,
            "epx_m": fix.epx_m,
            "epy_m": fix.epy_m,
            "eps_m_s": fix.eps_m_s,
            "satellites_used": fix.satellites_used,
            "satellites_visible": fix.satellites_visible,
            "device": fix.device,
            "status": fix.status,
            "connected": self.__connected,
            "last_error": self.__last_error,
        }

    def get_lat_lon(self) -> tuple[float | None, float | None]:
        """
        Return the latest latitude and longitude pair.
        """
        fix = self.get_fix()
        return fix.latitude, fix.longitude

    def has_fix(self, minimum_mode: int = 2) -> bool:
        """
        Report whether the cached fix meets the requested mode threshold.
        """
        return self.get_fix().mode >= minimum_mode

    def __reader_loop(self) -> None:
        while self.__running:
            try:
                self.update_once()
            except Exception as exc:
                self.__last_error = str(exc)
                self.close()
                if self.__verbose and self.__running:
                    print(f"GPS reader reconnecting after error: {exc}")
                if self.__running:
                    time.sleep(self.__reconnect_delay_sec)

    def __handle_report(self, report: Any) -> None:
        report_class = gpsd_report_get(report, "class")
        if report_class == "TPV":
            self.__update_from_tpv(report)
            return

        if report_class == "SKY":
            self.__update_from_sky(report)
            return

        if report_class == "DEVICE":
            if self.__verbose:
                printable = gpsd_report_to_dict(report)
                print(f"gpsd device event: {printable}")

    def __update_from_tpv(self, report: Any) -> None:
        mode = int(gpsd_report_get(report, "mode", 0) or 0)
        timestamp = gpsd_report_get(report, "time")
        latitude = safe_float(gpsd_report_get(report, "lat"))
        longitude = safe_float(gpsd_report_get(report, "lon"))
        altitude_hae_m = safe_float(gpsd_report_get(report, "altHAE"))
        altitude_msl_m = safe_float(gpsd_report_get(report, "altMSL"))
        speed_m_s = safe_float(gpsd_report_get(report, "speed"))
        track_deg = safe_float(gpsd_report_get(report, "track"))
        climb_m_s = safe_float(gpsd_report_get(report, "climb"))
        eph_m = safe_float(gpsd_report_get(report, "eph"))
        epv_m = safe_float(gpsd_report_get(report, "epv"))
        epx_m = safe_float(gpsd_report_get(report, "epx"))
        epy_m = safe_float(gpsd_report_get(report, "epy"))
        eps_m_s = safe_float(gpsd_report_get(report, "eps"))
        device = gpsd_report_get(report, "device")
        status = gpsd_report_get(report, "status")

        with self.__lock:
            self.__last_fix.timestamp = timestamp
            self.__last_fix.timestamp_datetime = parse_gpsd_time(timestamp)
            self.__last_fix.mode = mode
            self.__last_fix.mode_name = fix_mode_label(mode)
            if latitude is not None:
                self.__last_fix.latitude = latitude
            if longitude is not None:
                self.__last_fix.longitude = longitude
            if altitude_hae_m is not None:
                self.__last_fix.altitude_hae_m = altitude_hae_m
            if altitude_msl_m is not None:
                self.__last_fix.altitude_msl_m = altitude_msl_m
            if speed_m_s is not None:
                self.__last_fix.speed_m_s = speed_m_s
            if track_deg is not None:
                self.__last_fix.track_deg = track_deg
            if climb_m_s is not None:
                self.__last_fix.climb_m_s = climb_m_s
            if eph_m is not None:
                self.__last_fix.eph_m = eph_m
            if epv_m is not None:
                self.__last_fix.epv_m = epv_m
            if epx_m is not None:
                self.__last_fix.epx_m = epx_m
            if epy_m is not None:
                self.__last_fix.epy_m = epy_m
            if eps_m_s is not None:
                self.__last_fix.eps_m_s = eps_m_s
            if device is not None:
                self.__last_fix.device = device
            if status is not None:
                self.__last_fix.status = status

        if self.__verbose:
            fix = self.get_fix()
            if fix.has_2d_fix:
                print(
                    "GPS TPV "
                    f"mode={fix.mode_name} lat={fix.latitude:.6f} lon={fix.longitude:.6f} "
                    f"speed={fix.speed_m_s}"
                )
            else:
                print(f"GPS TPV mode={fix.mode_name} waiting for a valid fix")

    def __update_from_sky(self, report: Any) -> None:
        satellites = gpsd_report_get(report, "satellites", []) or []
        with self.__lock:
            self.__last_fix.satellites_visible = len(satellites)
            self.__last_fix.satellites_used = safe_count_used_satellites(satellites)


if __name__ == "__main__":
    gps_system = GPS_System(verbose=True)
    gps_system.start()

    try:
        while True:
            print(gps_system.get_data())
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped GPS test loop.")
    finally:
        gps_system.stop()
