"""
Plotting.py - Plot numeric telemetry columns from a CSV log.

This module loads a CSV file with a ``time`` or ``runtime`` column and plots each
numeric column against the time axis. It is intended for quick inspection of
logged telemetry data from the RC car control system.

Usage:
    python Plotting.py [csv_file] [--no-show]

Arguments:
    csv_file : str
        Path to a CSV file. Defaults to ``./data/data.csv``.
    --no-show : bool
        Build plots without opening an interactive window.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


DEFAULT_CSV_PATH = Path('.') / 'data' / 'data.csv'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Plot numeric telemetry columns from a CSV log.'
    )
    parser.add_argument(
        'csv_file',
        nargs='?',
        type=Path,
        default=DEFAULT_CSV_PATH,
        help='CSV file to plot. Defaults to ./data/data.csv.',
    )
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Create plots without opening an interactive window.',
    )
    return parser.parse_args()


def load_dataframe(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise FileNotFoundError(f'CSV file not found: {csv_path}')

    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f'CSV contains no data rows: {csv_path}')
    if 'time' not in df.columns and 'runtime' not in df.columns:
        raise ValueError("CSV must contain a 'time' or 'runtime' column.")
    return df


def get_time_series(df: pd.DataFrame) -> pd.Series:
    if 'time' in df.columns:
        return pd.to_numeric(df['time'], errors='coerce')
    return pd.to_numeric(df['runtime'], errors='coerce')


def plot_dataframe(df: pd.DataFrame, csv_path: Path) -> None:
    time_series = get_time_series(df)
    for column in df.columns:
        if column in {'time', 'runtime'}:
            continue

        if not pd.api.types.is_numeric_dtype(df[column]):
            continue

        values = pd.to_numeric(df[column], errors='coerce')
        if values.isna().all():
            continue

        plt.figure(figsize=(10, 4))
        plt.plot(time_series, values)
        plt.title(f'{column} ({csv_path.name})')
        plt.xlabel('time')
        plt.ylabel(column)
        plt.grid(True)
        plt.tight_layout()


def main() -> None:
    args = parse_args()
    csv_path = args.csv_file
    if csv_path.is_dir():
        csv_path = csv_path / 'data.csv'

    df = load_dataframe(csv_path)
    plot_dataframe(df, csv_path)

    if args.no_show:
        plt.close('all')
        return

    plt.show()


if __name__ == '__main__':
    main()
