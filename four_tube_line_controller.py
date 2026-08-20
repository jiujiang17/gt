"""Tube liquid-level controller with coordinated two-arm actions.

Run this script, select one to ten effective tube ROIs, then press Esc.
Press Enter after each dragged ROI to confirm it. The ROIs are sorted from
left to right and mapped to xiao.line_1 through xiao.line_5. A tube that stays below the limit for the configured duration
is queued once.  Lines 1--10 belong to the right arm.  Each completed
right-arm action schedules its paired left-arm line (line N -> line N + 10).
Left-arm tasks have higher queue priority.  Because the camera is mounted on
the left arm, image analysis pauses while a left-arm task is moving; the camera
handle and the last detection records remain alive and are reused afterwards.
"""

from __future__ import annotations

import argparse
import importlib
import json
import queue
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

from opencv_tube_camera_level import (
    configure_camera_exposure,
    open_capture,
    parse_camera_source,
)
from opencv_tube_level import (
    Rect,
    clamp_roi,
    fill_ratio_from_level,
    find_liquid_level,
    fit_for_display,
)
from opencv_tube_video_level import (
    find_central_backlight_panel,
    roi_from_panel_coordinates,
    roi_to_panel_coordinates,
)


MAX_TUBES = 10
COLORS = ((0, 255, 0), (255, 255, 0), (255, 0, 255), (0, 165, 255), (0, 128, 255))
IDLE = "IDLE"
QUEUED = "QUEUED"
RUNNING = "RUNNING"


# 功能：在首帧中选择一到四个试管 ROI，并按从左到右顺序编号。
# 参数：frame 为原始首帧；maximum_width/height 为选择窗口显示上限。
# 返回：长度为 1~4 的 Rect 列表；没有确认任何试管时抛出 RuntimeError。
def select_tube_rois(frame: np.ndarray, maximum_width: int, maximum_height: int) -> list[Rect]:
    shown, scale = fit_for_display(frame, maximum_width, maximum_height)
    window_name = "蛋白纯化实验:Drag ROI, Enter confirm, Esc start"
    selected: list[tuple[int, int, int, int]] = []
    drag_start: Optional[tuple[int, int]] = None
    drag_end: Optional[tuple[int, int]] = None
    pending: Optional[tuple[int, int, int, int]] = None

    def on_mouse(event: int, x: int, y: int, _: int, __: object) -> None:
        nonlocal drag_start, drag_end, pending
        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < MAX_TUBES:
            drag_start = (x, y)
            drag_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
            drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
            drag_end = (x, y)
            left, right = sorted((drag_start[0], drag_end[0]))
            top, bottom = sorted((drag_start[1], drag_end[1]))
            if right - left >= 4 and bottom - top >= 8:
                pending = (left, top, right - left, bottom - top)
            drag_start = None
            drag_end = None

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, on_mouse)
    while True:
        canvas = shown.copy()
        for index, (x, y, width, height) in enumerate(selected):
            color = COLORS[index]
            cv2.rectangle(canvas, (x, y), (x + width, y + height), color, 2)
            cv2.putText(canvas, "T%d" % (index + 1), (x + 3, y + 18), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, color, 2)
        if pending is not None:
            x, y, width, height = pending
            cv2.rectangle(canvas, (x, y), (x + width, y + height), (255, 255, 255), 1)
            cv2.putText(canvas, "Enter confirm", (x + 3, y + 18), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.45, (255, 255, 255), 1)
        if drag_start is not None and drag_end is not None:
            cv2.rectangle(canvas, drag_start, drag_end, (255, 255, 255), 1)

        cv2.putText(canvas, "Drag ROI, Enter confirm, ESC start", (12, 28), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(canvas, "Selected: %d / %d  Z undo  R reset" % (len(selected), MAX_TUBES), (12, 54), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(window_name, canvas)

        key = cv2.waitKey(20) & 0xFF
        if key in (13, 32) and pending is not None and len(selected) < MAX_TUBES:
            selected.append(pending)
            pending = None
        if key == 27 and selected:
            break
        if key in (ord("z"), 8, 127):
            if pending is not None:
                pending = None
            elif selected:
                selected.pop()
        if key in (ord("r"), ord("R")):
            selected.clear()
            pending = None
        if key in (ord("q"),):
            cv2.destroyWindow(window_name)
            raise RuntimeError("已取消试管框选。")

    cv2.destroyWindow(window_name)
    if not selected:
        raise RuntimeError("至少需要确认 1 根试管后才能按 Esc 开始。")
    # 保留按 Enter 确认的框选顺序：第一个确认的框是 T1 / line_1。
    rois = [tuple(int(round(value / scale)) for value in roi) for roi in selected]
    return rois


# 功能：安全加载 xiao.py 中的 line_1 到 line_4，并延后机械臂断开操作。
# 参数：无；xiao.py 必须位于工程目录，且 Robotic_Arm SDK 必须可导入。
# 返回：(xiao 模块, 四个可调用 line 函数元组)；加载失败时抛出异常。
def load_xiao_line_functions() -> tuple[object, tuple[Callable[[], None], ...]]:
    from Robotic_Arm.rm_robot_interface import RoboticArm

    original_delete = RoboticArm.rm_delete_robot_arm

    def defer_delete(_: object) -> None:
        """Keep xiao.py's module-level cleanup from closing the live connection."""

    RoboticArm.rm_delete_robot_arm = defer_delete
    try:
        xiao_module = importlib.import_module("xiao")
    finally:
        RoboticArm.rm_delete_robot_arm = original_delete

    actions = tuple(getattr(xiao_module, "line_%d" % index) for index in range(1, 21))
    if not all(callable(action) for action in actions):
        raise RuntimeError("xiao.py 必须定义可调用的 line_1 至 line_20。")
    return xiao_module, actions


class LineActionWorker:
    """Single worker that prioritizes left-arm work and schedules paired tasks."""

    # 功能：初始化串行动作队列并启动后台工作线程。
    # 参数：actions 按已选试管编号顺序保存 line_1 到 line_20 的前 N 个函数。
    # 返回：无；对象创建后线程立即等待队列任务。
    def __init__(self, actions: tuple[Callable[[], None], ...], left_trigger_count: int = 1) -> None:
        if len(actions) != 20:
            raise ValueError("actions must contain line_1 through line_20")
        if left_trigger_count <= 0:
            raise ValueError("left_trigger_count must be positive")
        self._actions = actions
        self._left_trigger_count = left_trigger_count
        self._queue: queue.PriorityQueue[tuple[int, int, Optional[int]]] = queue.PriorityQueue()
        self._lock = threading.Lock()
        self._states = [IDLE] * 20
        self._errors: list[Optional[str]] = [None] * 20
        self._right_completed = [0] * 10
        self._sequence = 0
        self._running_line: Optional[int] = None
        self._thread = threading.Thread(target=self._run, name="line-action-worker", daemon=True)
        self._thread.start()

    # 功能：将一根试管对应的 line 动作放入串行队列。
    # 参数：tube_index 是 0~3 的内部试管索引。
    # 返回：成功入队返回 True；该管已排队或执行中时返回 False。
    def enqueue(self, tube_index: int) -> bool:
        """Queue a right-arm line by its zero-based tube index."""
        return self._enqueue_line(tube_index + 1)

    def _enqueue_line(self, line_number: int) -> bool:
        if not 1 <= line_number <= 20:
            raise ValueError("line_number must be between 1 and 20")
        index = line_number - 1
        with self._lock:
            if self._states[index] != IDLE:
                return False
            self._states[index] = QUEUED
            self._errors[index] = None
            sequence = self._sequence
            self._sequence += 1
        # Smaller priority is dequeued first: left arm (11--20) precedes right arm.
        priority = 0 if line_number > 10 else 1
        self._queue.put((priority, sequence, line_number))
        return True

    # 功能：读取指定试管的机械臂任务状态和最近错误。
    # 参数：tube_index 是 0~3 的内部试管索引。
    # 返回：(IDLE/QUEUED/RUNNING 状态字符串, 错误文本或 None)。
    def state(self, tube_index: int) -> tuple[str, Optional[str]]:
        with self._lock:
            return self._states[tube_index], self._errors[tube_index]

    def camera_paused(self) -> bool:
        """Whether the left arm is currently moving the camera."""
        with self._lock:
            return self._running_line is not None and self._running_line > 10

    # 功能：串行执行队列中的阻塞 line 函数，仅由后台线程调用。
    # 参数：无；任务从内部 Queue 取出。
    # 返回：无；每个任务结束后将对应试管状态恢复为 IDLE。
    def _run(self) -> None:
        while True:
            _, _, line_number = self._queue.get()
            if line_number is None:
                return
            line_index = line_number - 1

            with self._lock:
                self._states[line_index] = RUNNING
                self._running_line = line_number
            print("开始执行 line_%d。" % line_number, flush=True)
            succeeded = False
            try:
                self._actions[line_index]()
                succeeded = True
            except Exception as error:  # Keep video detection alive if a robot command fails.
                with self._lock:
                    self._errors[line_index] = "%s: %s" % (type(error).__name__, error)
                print("line_%d 执行失败：%s" % (line_number, error), flush=True)
            finally:
                with self._lock:
                    self._states[line_index] = IDLE
                    self._running_line = None
                print("line_%d 执行结束。" % line_number, flush=True)

            if succeeded and line_number <= 10:
                with self._lock:
                    right_index = line_number - 1
                    self._right_completed[right_index] += 1
                    should_trigger_left = self._right_completed[right_index] % self._left_trigger_count == 0
                if should_trigger_left and self._enqueue_line(line_number + 10):
                    print(
                        "line_%d 已成功完成 %d 次，已加入高优先级 line_%d。"
                        % (line_number, self._right_completed[right_index], line_number + 10),
                        flush=True,
                    )

    # 功能：等待已开始的动作安全结束并停止后台线程。
    # 参数：无。
    # 返回：无；不会强制中断正在执行的机械臂动作。
    def close(self) -> None:
        with self._lock:
            sequence = self._sequence
            self._sequence += 1
        self._queue.put((2, sequence, None))
        self._thread.join()


# 功能：绘制一根试管的状态，并生成对应 JSON 检测记录。
# 参数：frame 会被原地绘制；roi 为当前管框；last_record 为冻结时保留的数据；其余参数描述当前动作和液面结果。
# 返回：该试管的最新或冻结检测记录字典。
def draw_tube(
    frame: np.ndarray,
    tube_index: int,
    roi: Rect,
    limit: float,
    action_state: str,
    action_error: Optional[str],
    last_record: Optional[dict[str, object]],
    level_y: Optional[int] = None,
    confidence: float = 0.0,
    fill_ratio: Optional[float] = None,
) -> dict[str, object]:
    x, y, width, height = roi
    color = COLORS[tube_index % len(COLORS)]
    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
    limit_y = y + int(height * (1.0 - limit))
    cv2.line(frame, (x, limit_y), (x + width, limit_y), (0, 0, 255), 2)
    cv2.putText(frame, "%.0f%%" % (limit * 100), (x + 3, limit_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)

    if action_state != IDLE:
        record = dict(last_record or {})
        record.update(
            {
                "tube_index": tube_index + 1,
                "roi": list(roi),
                "recognition_paused": True,
                "action_state": action_state,
                "action_error": action_error,
            }
        )
        label = "T%d %s" % (tube_index + 1, action_state)
        cv2.putText(frame, label, (x + 3, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        return record

    if level_y is not None:
        level_y = max(y, min(level_y, y + height))
        cv2.line(frame, (x, level_y), (x + width, level_y), color, 2)

    state = "NO LEVEL" if fill_ratio is None else ("LOW" if fill_ratio < limit else "OK")
    label = "T%d %s" % (tube_index + 1, "--" if fill_ratio is None else "%.1f%%" % (fill_ratio * 100))
    cv2.putText(frame, label, (x + 3, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
    cv2.putText(frame, state, (x + 3, y + 39), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 2)
    return {
        "tube_index": tube_index + 1,
        "roi": list(roi),
        "liquid_y": level_y,
        "fill_ratio": fill_ratio,
        "confidence": confidence,
        "recognition_paused": False,
        "action_state": IDLE,
        "action_error": action_error,
        "limit_reached": bool(fill_ratio is not None and fill_ratio >= limit),
    }


# 功能：将当前控制状态写入 JSON，便于上位机读取和故障追溯。
# 参数：path 为 JSON 路径；summary 为可序列化的完整运行状态字典。
# 返回：无；未传 path 时不写入文件。
def write_json(path: Optional[str], summary: dict[str, object]) -> None:
    if not path:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


# 功能：四根试管液面检测和 line_1~line_4 串行控制的命令行入口。
# 参数：从命令行读取相机、曝光、报警比例、持续时间、ROI 选择、显示、录像和 JSON 设置。
# 返回：无；实时显示检测结果，退出时安全等待动作完成并断开机械臂连接。
def main() -> None:
    parser = argparse.ArgumentParser(description="Four-tube liquid-level controller")
    parser.add_argument("source", nargs="?", default="0", help="camera index or RTSP URL")
    parser.add_argument("--backend", choices=("auto", "dshow", "msmf"), default="dshow")
    parser.add_argument("--limit", type=float, default=0.30, help="low-level threshold, default 0.30")
    parser.add_argument("--duration", type=float, default=1.0, help="continuous low-level duration in seconds")
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.20,
        help="minimum liquid-level confidence required to trigger, default 0.20",
    )
    parser.add_argument("--empty-zone", type=float, default=0.18, help="bottom ratio treated as empty")
    parser.add_argument("--auto-exposure", choices=("keep", "auto", "manual"), default="keep")
    parser.add_argument("--exposure", type=float)
    parser.add_argument("--gain", type=float)
    parser.add_argument("--warmup-frames", type=int, default=45)
    parser.add_argument("--record", help="annotated MP4 output")
    parser.add_argument("--json", dest="json_path", default="output/four_tube_line_control.json")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--display-width", type=int, default=1280)
    parser.add_argument("--display-height", type=int, default=720)
    args = parser.parse_args()

    if not 0.0 < args.limit < 1.0:
        parser.error("--limit must be between 0 and 1")
    if args.duration <= 0.0 or args.fps <= 0.0 or args.warmup_frames < 0:
        parser.error("--duration and --fps must be positive; --warmup-frames cannot be negative")
    if not 0.0 <= args.min_confidence <= 1.0:
        parser.error("--min-confidence must be between 0 and 1")
    if not 0.0 <= args.empty_zone < 1.0:
        parser.error("--empty-zone must be between 0 and 1")

    capture = open_capture(parse_camera_source(args.source), args.backend)
    if not capture.isOpened():
        raise RuntimeError("无法打开视频源：%s" % args.source)
    configure_camera_exposure(capture, args.auto_exposure, args.exposure, args.gain)

    for _ in range(args.warmup_frames):
        ok, _ = capture.read()
        if not ok:
            capture.release()
            raise RuntimeError("相机预热阶段读取失败")
    ok, frame = capture.read()
    if not ok or frame is None:
        capture.release()
        raise RuntimeError("无法读取首帧")

    initial_panel = find_central_backlight_panel(frame)
    selected_rois = [clamp_roi(roi, frame.shape) for roi in select_tube_rois(frame, args.display_width, args.display_height)]
    relative_rois = (
        [roi_to_panel_coordinates(roi, initial_panel) for roi in selected_rois]
        if initial_panel is not None
        else None
    )
    rois = selected_rois
    if initial_panel is None:
        print("首帧未识别到背光板：已保存四根试管框，等待后续画面捕获背光板。", flush=True)
    else:
        print("首帧背光板已识别，四根试管已完成相对板标定。", flush=True)

    xiao_module, all_actions = load_xiao_line_functions()
    worker = LineActionWorker(all_actions)
    print(
        "已选择 %d 根试管，并按从左到右映射到 line_1 至 line_%d；每条右臂线每成功 1 次即触发对应左臂线。"
        % (len(rois), len(rois)),
        flush=True,
    )

    writer: Optional[cv2.VideoWriter] = None
    if args.record:
        output = Path(args.record)
        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps,
            (frame.shape[1], frame.shape[0]),
        )
        if not writer.isOpened():
            worker.close()
            xiao_module.arm.rm_delete_robot_arm()
            capture.release()
            raise RuntimeError("无法创建录像文件：%s" % output)

    low_started: list[Optional[float]] = [None] * len(rois)
    triggered_while_low = [False] * len(rois)
    last_records: list[Optional[dict[str, object]]] = [None] * len(rois)
    frame_index = 0
    camera_pause_started: Optional[float] = None

    try:
        while frame is not None:
            now = time.monotonic()
            camera_paused = worker.camera_paused()
            if camera_paused:
                if camera_pause_started is None:
                    camera_pause_started = now
                    print("左臂动作中：已暂停影像识别，保留相机连接和历史识别结果。", flush=True)
                detected_panel = None
            else:
                if camera_pause_started is not None:
                    paused_seconds = now - camera_pause_started
                    low_started = [
                        None if started is None else started + paused_seconds
                        for started in low_started
                    ]
                    camera_pause_started = None
                    print("左臂动作完成：已恢复影像识别。", flush=True)
                detected_panel = find_central_backlight_panel(frame)
            result = frame.copy()
            if camera_paused:
                cv2.putText(
                    result,
                    "LEFT ARM MOVING: IMAGE ANALYSIS PAUSED",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )
            elif detected_panel is not None:
                if relative_rois is None:
                    # The first usable panel may arrive after manual ROI
                    # selection while exposure is still settling.
                    relative_rois = [roi_to_panel_coordinates(roi, detected_panel) for roi in rois]
                    print("背光板已捕获，四根试管开始按相对板坐标跟踪。", flush=True)
                rois = [
                    clamp_roi(roi_from_panel_coordinates(relative_roi, detected_panel), frame.shape)
                    for relative_roi in relative_rois
                ]
                panel_x, panel_y, panel_w, panel_h = detected_panel
                cv2.rectangle(result, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (255, 255, 0), 2)
                cv2.putText(result, "PANEL", (panel_x + 4, max(22, panel_y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            else:
                cv2.putText(
                    result,
                    "PANEL NOT FOUND: static ROI mode",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

            current_records: list[dict[str, object]] = []
            for tube_index, roi in enumerate(rois):
                action_state, action_error = worker.state(tube_index)
                if camera_paused:
                    # Do not run any image algorithm while the camera-mounted left arm moves.
                    # last_records is intentionally retained as the camera's detection memory.
                    record = draw_tube(
                        result,
                        tube_index,
                        roi,
                        args.limit,
                        "CAMERA PAUSED",
                        None,
                        last_records[tube_index],
                    )
                    record["panel_detected"] = None
                    current_records.append(record)
                    continue
                if action_state != IDLE:
                    # 本轮触发已进入队列或正在执行。动作完成后恢复识别时，
                    # 允许 LOW/NO LEVEL 从零重新计时并再次入队。
                    low_started[tube_index] = None
                    triggered_while_low[tube_index] = False
                    record = draw_tube(
                        result,
                        tube_index,
                        roi,
                        args.limit,
                        action_state,
                        action_error,
                        last_records[tube_index],
                    )
                    record["panel_detected"] = detected_panel is not None
                    current_records.append(record)
                    continue

                level_y, confidence = find_liquid_level(frame, roi)
                raw_fill_ratio, _ = fill_ratio_from_level(roi, level_y, args.empty_zone)
                reliable = confidence >= args.min_confidence
                fill_ratio = raw_fill_ratio if reliable else None
                record = draw_tube(
                    result,
                    tube_index,
                    roi,
                    args.limit,
                    action_state,
                    action_error,
                    last_records[tube_index],
                    level_y=level_y if reliable else None,
                    confidence=confidence,
                    fill_ratio=fill_ratio,
                )
                record["panel_detected"] = detected_panel is not None
                current_records.append(record)
                last_records[tube_index] = record

                # 只有可靠检测到液面且高度达到 limit，才算安全状态。
                # LOW 与 NO LEVEL 都计入未达到 limit 的持续时间。
                if fill_ratio is not None and fill_ratio >= args.limit:
                    low_started[tube_index] = None
                    triggered_while_low[tube_index] = False
                elif not triggered_while_low[tube_index]:
                    if low_started[tube_index] is None:
                        low_started[tube_index] = now
                    elif now - low_started[tube_index] >= args.duration:
                        if worker.enqueue(tube_index):
                            triggered_while_low[tube_index] = True
                            low_started[tube_index] = None
                            print(
                                "T%d 持续 %.1f 秒未检测到液面达到 limit，已加入 line_%d 动作队列。"
                                % (tube_index + 1, args.duration, tube_index + 1),
                                flush=True,
                            )

            summary = {
                "source": args.source,
                "frame_index": frame_index,
                "limit_ratio": args.limit,
                "low_duration_seconds": args.duration,
                "min_confidence": args.min_confidence,
                "panel": list(detected_panel) if detected_panel is not None else None,
                "normalized_tube_rois": [list(roi) for roi in relative_rois] if relative_rois is not None else None,
                "tubes": current_records,
            }
            write_json(args.json_path, summary)

            if writer is not None:
                writer.write(result)
            shown, _ = fit_for_display(result, args.display_width, args.display_height)
            cv2.imshow("Four tube line controller", shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            ok, frame = capture.read()
            if not ok:
                frame = None
            frame_index += 1
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        worker.close()
        xiao_module.arm.rm_delete_robot_arm()


if __name__ == "__main__":
    main()