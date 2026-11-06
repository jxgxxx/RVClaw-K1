"""Cloud vision client for RVClaw meter readings.

The board sends one JPEG to an OpenAI-compatible chat-completions endpoint and
normalises the model response to the fields used by the existing meter APIs.
Secrets are read from a board-local file and are never logged.
"""
from __future__ import annotations

import base64
import json
import math
import os
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener


DEFAULT_BASE_URL = "https://api.labhuan.com/v1"
DEFAULT_MODEL = "deepseek-v4-flash"
DEFAULT_MIN_CONFIDENCE = 0.70
PROJECT_ROOT = Path(__file__).resolve().parents[3]
MAX_IMAGE_BYTES = 10 * 1024 * 1024
MAX_RESPONSE_BYTES = 1024 * 1024


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


# Do not forward the board's API credential to a redirected host.
urlopen = build_opener(_NoRedirect()).open


def _failure(status: str, reason: str) -> dict:
    return {"status": status, "value": None, "confidence": 0.0, "reason": reason}


def _key() -> str:
    path = Path(os.getenv("RVCLAW_METER_API_KEY_FILE", str(
        PROJECT_ROOT / "runtime/k1/secrets/meter_cloud_api_key"
    )))
    try:
        value = path.read_text(encoding="utf-8").strip()
    except OSError:
        value = ""
    return value


def enabled() -> bool:
    return os.getenv("RVCLAW_METER_CLOUD_ENABLED", "0").strip().lower() in {"1", "true", "yes"}


def _min_confidence() -> float:
    try:
        value = float(os.getenv(
            "RVCLAW_METER_MIN_CONFIDENCE", str(DEFAULT_MIN_CONFIDENCE)
        ))
        return max(0.0, min(1.0, value)) if math.isfinite(value) else DEFAULT_MIN_CONFIDENCE
    except (TypeError, ValueError):
        return DEFAULT_MIN_CONFIDENCE


def _settings() -> tuple[str, str, float]:
    base_url = os.getenv("RVCLAW_METER_CLOUD_BASE_URL", DEFAULT_BASE_URL).rstrip("/")
    url = urlsplit(base_url)
    if (url.scheme != "https" or not url.hostname or url.username or url.password
            or url.query or url.fragment):
        raise ValueError("cloud base URL must be HTTPS without embedded credentials")
    model = os.getenv("RVCLAW_METER_CLOUD_MODEL", DEFAULT_MODEL).strip()
    timeout = float(os.getenv("RVCLAW_METER_CLOUD_TIMEOUT", "45"))
    if not model or not math.isfinite(timeout) or not 0.1 <= timeout <= 60.0:
        raise ValueError("cloud model or timeout is invalid")
    return base_url, model, timeout


def readiness() -> dict:
    """Check local configuration only: no network, image capture, or inference."""
    try:
        base_url, model, timeout = _settings()
    except (ValueError, TypeError):
        return {"ready": False, "enabled": enabled(), "reason": "invalid cloud configuration",
                "network_checked": False}
    key_configured = bool(_key())
    return {"ready": enabled() and key_configured, "enabled": enabled(),
            "key_configured": key_configured, "base_url": base_url, "model": model,
            "timeout_s": timeout, "min_confidence": _min_confidence(),
            "network_checked": False}


def _prompt(meter_type: str, calibration: dict | None) -> str:
    if meter_type == "digital":
        task = ("Read the main numeric display of this digital meter. Ignore other text. "
                "Preserve every decimal place and leading zero in raw_text.")
    else:
        task = ("Read the pointer value of this analog gauge from its visible scale. "
                "raw_text must contain only that numeric reading, not tick labels or the model name.")
    schema = (
        ' Return only JSON: {"status":"stable|unreadable","value":number|null,'
        '"raw_text":string|null,"unit":string|null,"confidence":number,'
        '"angle_deg":number|null,"reason":string}. Do not guess.'
    )
    if calibration:
        schema += " Known scale: " + json.dumps(calibration, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    return task + schema


def _normalise(value, meter_type: str) -> dict:
    if not isinstance(value, dict):
        return {"status": "unreadable", "value": None, "confidence": 0.0,
                "reason": "cloud response was not a JSON object"}
    status = str(value.get("status") or "unreadable").strip().lower()
    if status not in {"stable", "unreadable"}:
        status = "unreadable"
    reading = value.get("value")
    if status != "stable":
        reading = None
    elif isinstance(reading, str):
        try:
            reading = float(reading.replace(",", "."))
        except ValueError:
            reading = None
            status = "unreadable"
    if (status == "stable" and (isinstance(reading, bool)
            or not isinstance(reading, (int, float))
            or not math.isfinite(reading))):
        reading = None
        status = "unreadable"
    if isinstance(reading, float) and reading.is_integer():
        reading = int(reading)
    try:
        confidence = float(value.get("confidence", 0.0))
        confidence = max(0.0, min(1.0, confidence)) if math.isfinite(confidence) else 0.0
    except (TypeError, ValueError):
        confidence = 0.0
    candidate_value = None
    candidate_raw_text = None
    if status == "stable" and confidence < _min_confidence():
        candidate_value = reading
        candidate_raw_text = value.get("raw_text") or str(reading)
        reading = None
        status = "low_confidence"
    if status == "stable":
        reason = ""
    elif status == "low_confidence":
        reason = "云端返回了候选读数，但置信度低于 %.0f%%" % (_min_confidence() * 100)
    else:
        reason = str(value.get("reason") or "云端未能识别出读数")
    result = {
        "kind": meter_type,
        "status": status,
        "value": reading,
        "raw_text": str(value.get("raw_text") or reading) if status == "stable" else None,
        "unit": str(value["unit"]) if value.get("unit") else None,
        "confidence": confidence,
        "reason": reason,
        "value_method": "cloud_vision_chat_completions",
        "model": os.getenv("RVCLAW_METER_CLOUD_MODEL", DEFAULT_MODEL),
    }
    if status == "low_confidence":
        result["candidate_value"] = candidate_value
        result["candidate_raw_text"] = str(candidate_raw_text)
    if meter_type == "pointer":
        try:
            angle = float(value["angle_deg"]) if value.get("angle_deg") is not None else None
            result["angle_deg"] = angle if angle is not None and math.isfinite(angle) else None
        except (TypeError, ValueError):
            result["angle_deg"] = None
    return result


def _read_jpeg(image_bytes: bytes, meter_type: str, calibration: dict | None, timeout_s=None) -> dict:
    if not enabled():
        return {"status": "inference_offline", "value": None, "confidence": 0.0,
                "reason": "cloud meter inference is disabled"}
    api_key = _key()
    if not api_key:
        return {"status": "inference_offline", "value": None, "confidence": 0.0,
                "reason": "cloud meter API key is not configured"}
    if (meter_type not in {"digital", "pointer"}
            or not isinstance(image_bytes, bytes)
            or not 4 <= len(image_bytes) <= MAX_IMAGE_BYTES
            or not image_bytes.startswith(b"\xff\xd8\xff")
            or (calibration is not None and not isinstance(calibration, dict))):
        return _failure("inference_error", "invalid meter type, JPEG, or calibration")
    try:
        base_url, model, timeout = _settings()
        if timeout_s is not None:
            supplied_timeout = float(timeout_s)
            if not math.isfinite(supplied_timeout) or supplied_timeout <= 0:
                raise ValueError("invalid request timeout")
            timeout = min(timeout, supplied_timeout)
        prompt = _prompt(meter_type, calibration)
    except (ValueError, TypeError):
        return _failure("inference_error", "invalid cloud configuration or calibration")
    payload = {
        "model": model,
        "temperature": 0,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(image_bytes).decode("ascii")}},
        ]}],
    }
    # Optional provider settings verified against the configured K3 gateway.
    # Keep defaults absent for compatibility with other chat-completions APIs.
    if os.getenv("RVCLAW_METER_CLOUD_THINKING") == "disabled":
        payload["thinking"] = {"type": "disabled"}
    if os.getenv("RVCLAW_METER_CLOUD_MAX_TOKENS"):
        token_limit = int(os.environ["RVCLAW_METER_CLOUD_MAX_TOKENS"])
        if not 64 <= token_limit <= 2048:
            return _failure("inference_error", "invalid cloud output token limit")
        payload["max_tokens"] = token_limit
    request = Request(
        base_url + "/chat/completions",
        data=json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8"),
        headers={"Authorization": "Bearer " + api_key, "Content-Type": "application/json", "User-Agent": "RVClaw-Meter/1.0"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            raw = response.read(MAX_RESPONSE_BYTES + 1)
        if len(raw) > MAX_RESPONSE_BYTES:
            return _failure("inference_error", "cloud response exceeds size limit")
        body = json.loads(raw.decode("utf-8"))
        content = (((body.get("choices") or [{}])[0].get("message") or {}).get("content") or "")
        if isinstance(content, list):
            content = "".join(str(part.get("text", "")) for part in content if isinstance(part, dict))
        content = str(content).strip()
        if content.startswith("```"):
            content = content.strip("`").replace("json\n", "", 1).strip()
        return _normalise(json.loads(content), meter_type)
    except HTTPError as error:
        return {"status": "inference_error", "value": None, "confidence": 0.0,
                "reason": f"cloud API HTTP {error.code}"}
    except (OSError, URLError, TimeoutError) as error:
        return {"status": "inference_error", "value": None, "confidence": 0.0,
                "reason": f"cloud API unavailable: {type(error).__name__}"}
    except (ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError) as error:
        return {"status": "inference_error", "value": None, "confidence": 0.0,
                "reason": f"invalid cloud response: {type(error).__name__}"}


def read_jpeg(image_bytes: bytes, meter_type: str, calibration: dict | None = None, *, timeout_s=None) -> dict:
    """One cloud candidate only; the caller owns consensus and alarm thresholds.

    No camera, local model, automatic retries, or routing policy is started here.
    The timeout is a socket timeout, not a mission-wide wall-clock deadline.
    """
    started = time.monotonic()
    try:
        result = _read_jpeg(image_bytes, meter_type, calibration, timeout_s)
    except (ValueError, TypeError, OverflowError):
        result = _failure("inference_error", "invalid cloud input or configuration")
    result["elapsed_ms"] = round((time.monotonic() - started) * 1000, 1)
    return result
