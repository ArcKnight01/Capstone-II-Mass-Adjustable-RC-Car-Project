from __future__ import annotations

from datetime import datetime, timezone
from typing import Any
import math

import numpy as np


def nmea_lat(lat_deg: float) -> str:
    """
    Convert decimal latitude to NMEA latitude format.

    Parameters
    ----------
    lat_deg : float
        Latitude in decimal degrees.

    Returns
    -------
    str
        Latitude formatted for NMEA messages.
    """
    val = np.abs(lat_deg)
    degval = np.floor(val)
    minflt = (val - degval) * 60
    minval = np.floor(minflt)
    muval = int((minflt - minval) * 1e6)
    return f"{int(degval):02d}{int(minval):02d}.{int(muval):06d}"


def nmea_lon(lon_deg: float) -> str:
    """
    Convert decimal longitude to NMEA longitude format.

    Parameters
    ----------
    lon_deg : float
        Longitude in decimal degrees.

    Returns
    -------
    str
        Longitude formatted for NMEA messages.
    """
    val = np.abs(lon_deg)
    degval = np.floor(val)
    minflt = (val - degval) * 60
    minval = np.floor(minflt)
    muval = int((minflt - minval) * 1e6)
    return f"{int(degval):03d}{int(minval):02d}.{int(muval):06d}"


def safe_float(value: Any) -> float | None:
    """
    Convert a gpsd field to a finite float or return ``None``.
    """
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def gpsd_report_get(report: Any, key: str, default: Any = None) -> Any:
    """
    Read a field from a gpsd report object or dictionary.
    """
    if isinstance(report, dict):
        return report.get(key, default)
    return getattr(report, key, default)


def gpsd_report_to_dict(report: Any) -> dict[str, Any]:
    """
    Best-effort conversion of a gpsd report to a plain dictionary.
    """
    if isinstance(report, dict):
        return dict(report)
    if hasattr(report, "keys"):
        try:
            return {key: gpsd_report_get(report, key) for key in report.keys()}
        except Exception:
            pass
    if hasattr(report, "__dict__"):
        return dict(report.__dict__)
    return {"repr": repr(report)}


def parse_gpsd_time(value: str | None) -> datetime | None:
    """
    Parse gpsd ISO8601 timestamps into timezone-aware UTC datetimes.
    """
    if not value:
        return None
    try:
        if value.endswith("Z"):
            return datetime.fromisoformat(value.replace("Z", "+00:00"))
        parsed = datetime.fromisoformat(value)
        if parsed.tzinfo is None:
            return parsed.replace(tzinfo=timezone.utc)
        return parsed
    except ValueError:
        return None


def fix_mode_label(mode: int | None) -> str:
    """
    Translate gpsd mode integers to a readable label.
    """
    labels = {
        0: "unknown",
        1: "no_fix",
        2: "2d",
        3: "3d",
    }
    return labels.get(int(mode or 0), "unknown")


def safe_count_used_satellites(satellites: list[Any]) -> int:
    """
    Count the number of satellites flagged as used in a SKY report.
    """
    count = 0
    for satellite in satellites:
        used = gpsd_report_get(satellite, "used", False)
        if bool(used):
            count += 1
    return count
