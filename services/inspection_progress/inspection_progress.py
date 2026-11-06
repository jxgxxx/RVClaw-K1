"""RVClaw 巡检进度推导（纯逻辑层）。

本模块**不做任何 I/O**：输入是已经从板端读到的原始数据（任务行、到点事件、跑车归档），
输出是页面直接可渲染的进度快照。这样做的原因很明确 —— 板端的 I/O 层（spool 扫描、
SQLite WAL 读取）没法在开发机上复现，但"开始 → 到达第 N 个巡检点 → 返回起点 → 结束"
这套状态推导必须能被离线单测覆盖。

数据口径（来自 2026-09-18 对板端的只读核查）：

* 任务生命周期：`voice_tasks.sqlite3` 的 `tasks.state` + `events.type`
  （`TASK_CREATED / TASK_ACCEPTED / TASK_STARTED / TASK_COMPLETED / TASK_ABORTED`）。
  `TASK_STARTED` 的 payload 里带 `route`，例如 `"A,B,C,ORIGIN"`。
* 到点：spool 里的 `waypoint_arrived`（`waypoint` ∈ {A,B,C}，`status=PASS`）。
  ⚠️ 这些文件在语音服务播报成功后会立刻被 `unlink`，寿命只有 2~4 秒 —— 所以消费方
  必须高频扫描并自己记账，不能指望"事后去 spool 里翻"。
* 兜底：跑车归档 `<run_dir>/trace.json` 的 `completed_waypoints`，只在整个流程跑完时写一次，
  是权威结果，用来回填/纠偏前面可能漏掉的到点记录。
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Sequence

SCHEMA_VERSION = 1

#: A/B/C 三个巡检点的中文序数词叫法。
CN_ORDINALS = (
    "第一个",
    "第二个",
    "第三个",
    "第四个",
    "第五个",
    "第六个",
    "第七个",
    "第八个",
)

#: 非巡检点的特殊目标名。
ORIGIN_NAMES = {"ORIGIN", "START", "HOME", "BASE"}

#: 有到点事件的巡检点（与路线客户端的 `publish_arrival_event` 白名单一致）。
ANNOUNCED_WAYPOINTS = {"A", "B", "C"}

#: 任务终态。
TERMINAL_STATES = {"completed", "aborted", "failed"}

#: spool 里代表"本次巡检结束"的事件（成功一件、失败一件）。
TERMINAL_EVENT_KINDS = ("inspection_finished", "inspection_failed")

STATE_TEXT = {
    "idle": "待机",
    "created": "已创建",
    "accepted": "已受理",
    "running": "巡检中",
    "completed": "巡检完成",
    "aborted": "已中止",
    "failed": "巡检失败",
}

STEP_STATUS_TEXT = {
    "done": "已完成",
    "active": "进行中",
    "pending": "等待中",
    "failed": "异常",
}


# --------------------------------------------------------------------------------------
# 解析
# --------------------------------------------------------------------------------------


def parse_route(value: Any) -> List[str]:
    """把 `route` 规范成目标名列表。

    板端的 `TASK_STARTED.payload.route` 是 `"A,B,C,ORIGIN"` 这种逗号串；有的地方也可能是
    已经是列表。两种都吃，并且去掉空项、统一大写。
    """

    if value is None:
        return []
    if isinstance(value, (list, tuple, set)):
        items = [str(item).strip() for item in value]
    else:
        items = [part.strip() for part in str(value).replace(";", ",").split(",")]
    return [item.upper() for item in items if item]


def waypoint_label(name: Any) -> str:
    """巡检点的展示名：A → 「第一个巡检点（A）」，ORIGIN → 「返回起点」."""

    upper = str(name).strip().upper()
    if upper in ANNOUNCED_WAYPOINTS:
        index = {"A": 1, "B": 2, "C": 3}[upper]
        return "%s巡检点（%s）" % (CN_ORDINALS[index - 1], upper)
    if upper in ORIGIN_NAMES:
        return "返回起点（%s）" % upper
    return "巡检点 %s" % upper


def parse_spool_event(payload: Any) -> Optional[Dict[str, Any]]:
    """把 spool 里的一个 JSON 事件规范化；不认识的事件返回 None（直接忽略）。

    只接受会改变进度语义的事件，避免页面被无关事件驱动。
    """

    if not isinstance(payload, dict):
        return None
    kind = payload.get("event")
    timestamp = payload.get("timestamp")
    try:
        at = float(timestamp)
    except (TypeError, ValueError):
        at = None

    if kind == "waypoint_arrived":
        waypoint = str(payload.get("waypoint", "")).strip().upper()
        if payload.get("status") != "PASS" or waypoint not in ANNOUNCED_WAYPOINTS:
            return None
        return {"kind": "arrival", "waypoint": waypoint, "at": at}

    if kind in {"inspection_started", "inspection_finished", "inspection_failed"}:
        return {
            "kind": kind,
            "status": payload.get("status"),
            "at": at,
            "return_code": payload.get("return_code"),
        }

    return None


def arrival_map(events: Iterable[Dict[str, Any]]) -> Dict[str, float]:
    """把规范化后的事件流压成 `{巡检点名: 首次到达时间}`。

    同名事件只保留**最早**的一次：重试/重复发布不应该让时间戳往后跳。
    """

    result: Dict[str, float] = {}
    for event in events:
        if not isinstance(event, dict) or event.get("kind") != "arrival":
            continue
        waypoint = str(event.get("waypoint", "")).upper()
        at = event.get("at")
        if not waypoint or not isinstance(at, (int, float)):
            continue
        if waypoint not in result or at < result[waypoint]:
            result[waypoint] = float(at)
    return result


# --------------------------------------------------------------------------------------
# 推导
# --------------------------------------------------------------------------------------


def _step(step_id: str, kind: str, label: str) -> Dict[str, Any]:
    return {
        "id": step_id,
        "kind": kind,
        "label": label,
        "status": "pending",
        "status_text": STEP_STATUS_TEXT["pending"],
        "at": None,
    }


def build_steps(
    route: Sequence[str],
    arrivals: Dict[str, float],
    trace_completed: Sequence[str],
    state: str,
    *,
    started_at: Optional[float] = None,
    finished_at: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """推导完整的步骤时间轴。

    步骤顺序：开始巡检 → 各巡检点（按 route 顺序）→ 返回起点（若有）→ 巡检结束。

    路线为空时返回空时间轴 —— 与其凭空造一条"开始/结束"的两步轴，不如让页面明确
    显示"暂无巡检计划"。`route` 要么来自任务的 `TASK_STARTED.payload.route`，
    要么来自待机时的计划路线骨架。
    """

    if not route:
        return []

    completed = {str(name).strip().upper() for name in trace_completed}
    has_started = bool(started_at is not None or state in {"running"} | TERMINAL_STATES)

    steps: List[Dict[str, Any]] = [_step("start", "start", "开始巡检")]
    if has_started:
        steps[0]["status"] = "done"
        steps[0]["at"] = started_at

    points = [name for name in route if name not in ORIGIN_NAMES]
    for name in points:
        step = _step(name, "waypoint", waypoint_label(name))
        # 真实观测到的到点时刻优先：trace 只能证明"到过"，给不出"什么时候到"。
        # 所以两者都成立时，保留 arrivals 的时间戳，别被 trace 覆盖成 None。
        if name in arrivals:
            step["status"] = "done"
            step["at"] = arrivals[name]
        elif name in completed:
            step["status"] = "done"
        steps.append(step)

    if any(name in ORIGIN_NAMES for name in route):
        origin = _step("ORIGIN", "origin", waypoint_label("ORIGIN"))
        if "ORIGIN" in completed or state == "completed":
            origin["status"] = "done"
            origin["at"] = finished_at
        steps.append(origin)

    finish = _step("finish", "finish", "巡检结束")
    if state == "completed":
        finish["status"] = "done"
        finish["at"] = finished_at
    elif state in {"aborted", "failed"}:
        finish["status"] = "failed"
        finish["at"] = finished_at
        finish["label"] = "巡检中止" if state == "aborted" else "巡检失败"
    steps.append(finish)

    # 回填时间戳：trace 里没有逐点时间，所以完成但无到点记录的点只有状态、没有时刻。
    for step in steps:
        step["status_text"] = STEP_STATUS_TEXT[step["status"]]

    # 进行中：把第一个还没完成的步骤点亮（结束步骤不参与，它只在收尾时变化）。
    if state == "running":
        for step in steps[:-1]:
            if step["status"] == "pending":
                step["status"] = "active"
                step["status_text"] = STEP_STATUS_TEXT["active"]
                break

    return steps


def current_step_id(steps: Sequence[Dict[str, Any]], state: str) -> Optional[str]:
    """当前应高亮的步骤：巡检中＝第一个进行中的步骤。"""

    if state != "running":
        return None
    for step in steps:
        if step["status"] == "active":
            return step["id"]
    return None


def resolve_state(
    task_state: Optional[str],
    spool_events: Sequence[Dict[str, Any]] = (),
) -> str:
    """把任务库状态与 spool 事件合成页面要用的状态。

    spool 里的 `inspection_failed` 往往比任务库更早到达（任务库要等子进程退出码回写），
    所以这里让它优先把状态压成 failed。
    """

    for event in spool_events:
        if event.get("kind") == "inspection_failed":
            return "failed"
    if task_state:
        normalised = str(task_state).strip().lower()
        if normalised:
            return normalised
    return "idle"


def elapsed_seconds(
    started_at: Optional[float],
    finished_at: Optional[float],
    now: float,
) -> Optional[float]:
    """本次巡检用时：跑完了用实际区间，还在跑就用当前时刻推。"""

    if started_at is None:
        return None
    end = finished_at if finished_at is not None else now
    delta = float(end) - float(started_at)
    return delta if delta >= 0 else None


def first_terminal_event(
    events: Iterable[Dict[str, Any]],
    started_at: Optional[float],
    upper_bound: Optional[float] = None,
    grace_s: float = 5.0,
) -> Optional[Dict[str, Any]]:
    """本次巡检时间窗内**最早**的结束事件（`inspection_finished` / `inspection_failed`）。

    用途：给"没有 `finished_at` 的终态"补结束时间。板端失败这条路径**不写任务库的
    结束记录**（2026-09-20 实测：连续三次失败任务的 `finished_at`/`duration_s` 全是 null，
    而 completed/aborted 都有值），后果有两个 ——

    1. 页面拿本地钟外推"已用时"，失败态的时钟会一直走，看着像车还在跑；
    2. `_arrivals_for()` 的窗口右端不设界，**之后**别的活动的到点会被算进这次已死的巡检
       （实测：09:59 那次 13 秒就失败的巡检，把 11:38~11:40 的 A/B/C 到点吃掉了）。

    取**最早**而不是最新：一旦结束事件到了，窗口就该合拢，后面的到点不属于本次。
    回传事件本体（而不是只给时间戳），因为 `return_code` 也只在这个事件里。
    """

    lower = (started_at - grace_s) if isinstance(started_at, (int, float)) else None
    earliest: Optional[Dict[str, Any]] = None
    for event in events:
        if not isinstance(event, dict) or event.get("kind") not in TERMINAL_EVENT_KINDS:
            continue
        at = event.get("at")
        if not isinstance(at, (int, float)):
            continue
        if lower is not None and at < lower:
            continue
        if upper_bound is not None and at > upper_bound:
            continue
        if earliest is None or float(at) < float(earliest["at"]):
            earliest = event
    return dict(earliest) if earliest is not None else None


def summarise_run(
    *,
    run: Optional[Dict[str, Any]] = None,
    arrivals: Optional[Dict[str, float]] = None,
    trace: Optional[Dict[str, Any]] = None,
    history: Optional[Sequence[Dict[str, Any]]] = None,
    notes: Optional[Sequence[str]] = None,
    now: float = 0.0,
    default_route: Any = None,
    camera_stream_url: str = "/stream/astra.mjpg",
    host: str = "rvclaw-k3",
) -> Dict[str, Any]:
    """产出页面轮询的那份 JSON。

    `run` 的字段：`task_id / state / started_at / finished_at / route / run_dir /
    trace_file / return_code`，全部可选（没有任务时传 None 即得到待机快照）。

    `default_route` 用于**待机**时铺出计划路线的骨架步骤（例如 `"A,B,C,ORIGIN"`）——
    这样页面任何时候都能看到三个巡检点，而不是一片空白。全部为"等待中"，不伪造已完成。
    """

    run = dict(run or {})
    route = parse_route(run.get("route")) or parse_route(default_route)
    state = str(run.get("state") or "idle").strip().lower()
    started_at = run.get("started_at")
    finished_at = run.get("finished_at")

    trace = dict(trace or {})
    trace_completed = parse_route(trace.get("completed_waypoints"))

    steps = build_steps(
        route,
        arrivals or {},
        trace_completed,
        state,
        started_at=started_at,
        finished_at=finished_at,
    )

    done = sum(1 for step in steps if step["status"] == "done")
    total = len(steps)
    percent = int(round(done * 100.0 / total)) if total else 0

    payload: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": float(now),
        "host": host,
        "state": state,
        "state_text": STATE_TEXT.get(state, state),
        "task_id": run.get("task_id"),
        "route": route,
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_s": elapsed_seconds(started_at, finished_at, now),
        "route_duration_s": trace.get("duration_s"),
        "return_code": run.get("return_code"),
        "run_dir": run.get("run_dir"),
        "trace_file": run.get("trace_file"),
        "steps": steps,
        "current_step_id": current_step_id(steps, state),
        "progress": {"done": done, "total": total, "percent": percent},
        "history": [dict(item) for item in (history or [])],
        "notes": list(notes or []),
        "camera": {"stream_url": camera_stream_url},
    }
    return payload


def summarise_history(
    tasks: Iterable[Dict[str, Any]],
    traces_by_task: Optional[Dict[str, Dict[str, Any]]] = None,
    limit: int = 6,
) -> List[Dict[str, Any]]:
    """最近几次巡检的概要，供页面底部列表展示（新的在前）。"""

    traces_by_task = traces_by_task or {}
    items: List[Dict[str, Any]] = []
    for task in tasks:
        task_id = task.get("task_id")
        state = str(task.get("state") or "idle").strip().lower()
        trace = traces_by_task.get(task_id) or {}
        completed = parse_route(trace.get("completed_waypoints"))
        started_at = task.get("started_at")
        finished_at = task.get("finished_at")
        items.append(
            {
                "task_id": task_id,
                "state": state,
                "state_text": STATE_TEXT.get(state, state),
                "route": parse_route(task.get("route")),
                "completed_waypoints": completed,
                "started_at": started_at,
                "finished_at": finished_at,
                "duration_s": elapsed_seconds(started_at, finished_at, finished_at or 0.0),
                "return_code": task.get("return_code"),
            }
        )
    items.sort(key=lambda item: item.get("started_at") or 0.0, reverse=True)
    return items[:limit]


def idle_payload(
    *,
    now: float = 0.0,
    default_route: Any = None,
    camera_stream_url: str = "/stream/astra.mjpg",
) -> Dict[str, Any]:
    """没有任何任务时的快照（页面首屏、或板端还没跑过巡检）。"""

    return summarise_run(
        now=now, default_route=default_route, camera_stream_url=camera_stream_url
    )
