import argparse
import time
from pathlib import Path

import pandas as pd
import requests


DMI_BASE_URL = "https://opendataapi.dmi.dk/v2/metObs/collections/observation/items"
DEFAULT_STATION_ID = "06030"

DEFAULT_PARAMETER_MAP = {
    "air_temp": "temp_dry",
    "humidity": "humidity",
    "pressure": "pressure",
    "wind": "wind_speed",
    "precip": "precip_past10min",
    "lux": "radia_glob",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output-dir", default="output")
    parser.add_argument("--station-id", default=DEFAULT_STATION_ID)
    parser.add_argument("--time-column", default=None)
    parser.add_argument("--map", action="append", default=[])
    parser.add_argument("--sleep", type=float, default=0.25)
    return parser.parse_args()


def detect_time_column(df, explicit=None):
    if explicit:
        if explicit not in df.columns:
            raise ValueError(f"Time column '{explicit}' not found.")
        return explicit

    candidates = [
        "timestamp",
        "time",
        "datetime",
        "date",
        "created_at",
        "observed_at",
        "ts",
    ]

    lower_to_original = {col.lower(): col for col in df.columns}

    for candidate in candidates:
        if candidate in lower_to_original:
            return lower_to_original[candidate]

    for col in df.columns:
        parsed = pd.to_datetime(df[col], errors="coerce", utc=True)
        if parsed.notna().mean() >= 0.8:
            return col

    raise ValueError("Could not detect timestamp column. Use --time-column.")


def parse_mapping(overrides):
    mapping = dict(DEFAULT_PARAMETER_MAP)

    for item in overrides:
        if "=" not in item:
            raise ValueError(f"Invalid --map value '{item}'.")

        sensor_col, dmi_param = item.split("=", 1)
        sensor_col = sensor_col.strip()
        dmi_param = dmi_param.strip()

        if not sensor_col or not dmi_param:
            raise ValueError(f"Invalid --map value '{item}'.")

        mapping[sensor_col] = dmi_param

    return mapping


def load_sensor_period(csv_path, time_column):
    df = pd.read_csv(csv_path)
    time_col = detect_time_column(df, time_column)

    df[time_col] = pd.to_datetime(df[time_col], errors="coerce", utc=True)
    df = df.dropna(subset=[time_col])

    if df.empty:
        raise ValueError("No valid timestamps found.")

    return df[time_col].min(), df[time_col].max(), time_col


def dmi_datetime_interval(start, end):
    start_iso = start.strftime("%Y-%m-%dT%H:%M:%SZ")
    end_iso = end.strftime("%Y-%m-%dT%H:%M:%SZ")
    return f"{start_iso}/{end_iso}"


def fetch_dmi_parameter(station_id, parameter_id, start, end, sleep_seconds):
    params = {
        "stationId": station_id,
        "parameterId": parameter_id,
        "datetime": dmi_datetime_interval(start, end),
        "limit": 300000,
    }

    response = requests.get(DMI_BASE_URL, params=params, timeout=60)

    if response.status_code >= 400:
        raise RuntimeError(
            f"DMI API error for '{parameter_id}': "
            f"{response.status_code} {response.text[:500]}"
        )

    features = response.json().get("features", [])
    rows = []

    for feature in features:
        props = feature.get("properties", {})
        observed = props.get("observed")
        value = props.get("value")

        if observed is None or value is None:
            continue

        rows.append(
            {
                "time": observed,
                "station_id": props.get("stationId", station_id),
                "parameter_id": props.get("parameterId", parameter_id),
                "value": value,
            }
        )

    time.sleep(sleep_seconds)

    if not rows:
        return pd.DataFrame(columns=["time", "station_id", "parameter_id", "value"])

    df = pd.DataFrame(rows)
    df["time"] = pd.to_datetime(df["time"], errors="coerce", utc=True)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    df = df.dropna(subset=["time", "value"])

    return df.sort_values("time")


def fetch_all_dmi(station_id, parameters, start, end, sleep_seconds):
    frames = []

    for parameter in sorted(set(parameters)):
        print(f"Fetching {parameter}")

        df = fetch_dmi_parameter(
            station_id=station_id,
            parameter_id=parameter,
            start=start,
            end=end,
            sleep_seconds=sleep_seconds,
        )

        print(f"  rows: {len(df):,}")
        frames.append(df)

    if not frames:
        return pd.DataFrame(columns=["time", "station_id", "parameter_id", "value"])

    return pd.concat(frames, ignore_index=True)


def make_sensor_named_dmi(dmi_long, mapping):
    if dmi_long.empty:
        return pd.DataFrame(columns=["time"])

    dmi_wide = (
        dmi_long.pivot_table(
            index="time",
            columns="parameter_id",
            values="value",
            aggfunc="mean",
        )
        .reset_index()
        .sort_values("time")
    )

    dmi_wide.columns.name = None

    dmi_to_sensor = {dmi_param: sensor_col for sensor_col, dmi_param in mapping.items()}

    dmi_wide = dmi_wide.rename(columns=dmi_to_sensor)

    ordered_columns = ["time"]

    for sensor_col in mapping.keys():
        if sensor_col in dmi_wide.columns:
            ordered_columns.append(sensor_col)

    extra_columns = []

    for col in dmi_wide.columns:
        if col not in ordered_columns:
            extra_columns.append(col)

    return dmi_wide[ordered_columns + extra_columns]


def main():
    args = parse_args()

    csv_path = Path(args.csv_path)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    mapping = parse_mapping(args.map)
    start, end, time_col = load_sensor_period(csv_path, args.time_column)

    print(f"Time column: {time_col}")
    print(f"Period: {start} to {end}")
    print(f"Station: {args.station_id}")

    dmi_long = fetch_all_dmi(
        station_id=args.station_id,
        parameters=mapping.values(),
        start=start,
        end=end,
        sleep_seconds=args.sleep,
    )

    dmi_named = make_sensor_named_dmi(dmi_long, mapping)

    raw_long_path = output_dir / "dmi_only_raw_long.csv"
    named_path = output_dir / "dmi_only_aalborg_sensor_names.csv"

    dmi_long.to_csv(raw_long_path, index=False)
    dmi_named.to_csv(named_path, index=False)

    print(f"Wrote: {raw_long_path}")
    print(f"Wrote: {named_path}")


if __name__ == "__main__":
    main()
