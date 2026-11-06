#!/usr/bin/env python3
"""Explicit, independent cloud meter reader; does not start robot services."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from meter_gauge import cloud_meter

ROOT = Path(__file__).resolve().parents[2]
CONFIG_KEYS = frozenset({
    "RVCLAW_METER_CLOUD_ENABLED", "RVCLAW_METER_CLOUD_BASE_URL",
    "RVCLAW_METER_CLOUD_MODEL", "RVCLAW_METER_CLOUD_TIMEOUT",
    "RVCLAW_METER_API_KEY_FILE", "RVCLAW_METER_MIN_CONFIDENCE",
    "RVCLAW_METER_CLOUD_THINKING", "RVCLAW_METER_CLOUD_MAX_TOKENS",
    "RVCLAW_METER_HYBRID_ENABLED", "RVCLAW_METER_HYBRID_BUDGET_S",
    "RVCLAW_METER_HYBRID_FINAL_WAIT_S", "RVCLAW_METER_HYBRID_REQUEST_TIMEOUT_S",
})


def load_config(path: Path) -> None:
    """Read literal KEY=VALUE settings, without evaluating shell commands.

    Explicit process environment overrides the config file.
    """
    settings = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        key, value = key.strip(), value.strip()
        if not separator or key not in CONFIG_KEYS or key in settings:
            raise ValueError("invalid or duplicate cloud configuration setting")
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        settings[key] = value
    for key, value in settings.items():
        os.environ.setdefault(key, value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "configs/device/meter/cloud_meter.env")
    parser.add_argument("--check", action="store_true", help="local config check; no network")
    parser.add_argument("--image", type=Path, help="JPEG to send to the configured cloud provider")
    parser.add_argument("--meter-type", choices=("digital", "pointer"))
    parser.add_argument("--calibration", type=Path, help="optional scale-only JSON file")
    args = parser.parse_args(argv)
    if args.check and (args.image or args.meter_type or args.calibration):
        parser.error("--check cannot be combined with recognition arguments")
    if not args.check and (args.image is None or args.meter_type is None):
        parser.error("use --check, or provide both --image and --meter-type")
    try:
        load_config(args.config)
        if args.check:
            result = cloud_meter.readiness()
            code = 0 if result["ready"] else 1
        else:
            if not 4 <= args.image.stat().st_size <= cloud_meter.MAX_IMAGE_BYTES:
                raise ValueError("JPEG size must be between 4 bytes and 10 MiB")
            calibration = None
            if args.calibration:
                calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
                if not isinstance(calibration, dict):
                    raise ValueError("calibration must be a JSON object")
            result = cloud_meter.read_jpeg(args.image.read_bytes(), args.meter_type, calibration)
            code = 0 if result["status"] == "stable" else 1
    except (OSError, ValueError, TypeError):
        # Never include file contents or the credential in errors.
        result = {"status": "inference_error", "value": None, "confidence": 0.0,
                  "reason": "cannot read valid cloud configuration or input files"}
        code = 2
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
