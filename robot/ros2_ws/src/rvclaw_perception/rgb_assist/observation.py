"""Image-space target checks; never infer metres from an uncalibrated camera."""
import math


def alignment(bbox, width, height):
    unknown = {"status": "unknown", "reason": "no verified target geometry"}
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4 or min(width, height) <= 0:
        return unknown
    try:
        x0, y0, x1, y1 = map(float, bbox)
    except (TypeError, ValueError):
        return unknown
    if not all(map(math.isfinite, (x0, y0, x1, y1))) or x1 <= x0 or y1 <= y0:
        return unknown
    cx, cy = (x0+x1)/(2*width), (y0+y1)/(2*height)
    visible = 0 <= x0 < x1 < width and 0 <= y0 < y1 < height
    readable_size = min(x1-x0, y1-y0) >= 12 and x1-x0 >= 30
    return {"status": "visible" if visible and readable_size else "clipped_or_small",
            "bbox": [x0,y0,x1,y1], "center_offset_x": round(cx-.5,4),
            "center_offset_y": round(cy-.5,4),
            "width_fraction": round((x1-x0)/width,4),
            "image_side": "left" if cx < .4 else "right" if cx > .6 else "center",
            "metric_pose_available": False}


def trial_observation(result, shape):
    registration = result.get("reference_registration") or {}
    # A detector box alone is not proof of meter identity.
    bbox = registration.get("bbox")
    if (bbox is None and result.get('value_method')=='registered_scale_current_pointer'
            and len(result.get('notes',{}).get('static_scale_checks',[]))>=2):
        bbox=result.get('gauge_box')
    if bbox is None and registration.get("display_quad"):
        pts = registration["display_quad"]
        bbox = [min(p[0] for p in pts),min(p[1] for p in pts),
                max(p[0] for p in pts),max(p[1] for p in pts)]
    return {"alignment": alignment(bbox, shape[1], shape[0]),
            "landmark": {"status": "expected_template_matched" if bbox else "unknown",
                         "absolute_pose_available": False},
            "motion_authorized": False}
