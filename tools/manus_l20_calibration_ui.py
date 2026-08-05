#!/usr/bin/env python3
"""Qt-only operator workflow for capturing MANUS calibration data."""

from __future__ import annotations

import sys
from pathlib import Path
from math import cos, isfinite, sin
from typing import Any, Callable

from PyQt5.QtCore import QEasingCurve, QPoint, QPointF, QProcess, QPropertyAnimation, QSize, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QMovie, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QBoxLayout,
    QFrame,
    QGraphicsDropShadowEffect,
    QGraphicsOpacityEffect,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


BG = "#f5f5f7"
CARD = "#ffffff"
TEXT = "#1d1d1f"
MUTED = "#6e6e73"
BLUE = "#0071e3"
GREEN = "#248a3d"
RED = "#d70015"
ASSET_DIR = Path(__file__).resolve().parent / "assets"
BRIDGE_SCRIPT = Path(__file__).resolve().parent / "manus_calibration_ros_bridge.py"


def full_session_factory(**kwargs):
    from manus_l20_retarget.calibration_capture import FullCalibrationSession

    return FullCalibrationSession(**kwargs)


def contact_session_factory(**kwargs):
    from manus_l20_retarget.calibration_capture import FingertipContactCalibrationSession

    return FingertipContactCalibrationSession(**kwargs)


def card() -> QFrame:
    frame = QFrame()
    frame.setObjectName("card")
    frame.setFrameShape(QFrame.StyledPanel)
    shadow = QGraphicsDropShadowEffect(frame)
    shadow.setBlurRadius(22)
    shadow.setOffset(0, 4)
    shadow.setColor(QColor(0, 0, 0, 22))
    frame.setGraphicsEffect(shadow)
    return frame


def status_dot(ok: bool) -> QLabel:
    label = QLabel("●")
    label.setProperty("status", "ok" if ok else "bad")
    label.setFixedWidth(20)
    return label


class TaskThread(QThread):
    succeeded = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, operation: Callable[[], Any], parent: QWidget) -> None:
        super().__init__(parent)
        self.operation = operation

    def run(self) -> None:
        try:
            self.succeeded.emit(self.operation())
        except Exception as exc:  # The UI must surface ROS/SDK failures without closing.
            self.failed.emit(str(exc))


class GestureImage(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self._source = QPixmap()
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

    def set_image(self, path: Path) -> bool:
        self._source = QPixmap(str(path))
        self._refresh()
        return not self._source.isNull()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self) -> None:
        if self._source.isNull():
            self.clear()
            return
        side = max(1, min(self.width(), self.height()))
        self.setPixmap(
            self._source.scaled(
                side,
                side,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


class SystemPage(QWidget):
    topics_ready = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.rows: dict[str, tuple[QLabel, QLabel]] = {}
        self._probe_active = False
        self.probe = QProcess(self)
        self.probe.finished.connect(self._probe_finished)
        self.probe.errorOccurred.connect(self._probe_error)
        self.probe_timeout = QTimer(self)
        self.probe_timeout.setSingleShot(True)
        self.probe_timeout.timeout.connect(self._probe_timed_out)
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 28)
        root.setSpacing(18)

        title = QLabel("系统连接状态")
        title.setObjectName("pageTitle")
        root.addWidget(title)
        subtitle = QLabel("标定只依赖 ROS2 图和 MANUS 手套话题，不连接 L20 控制通道")
        subtitle.setObjectName("subtitle")
        root.addWidget(subtitle)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        for index, (key, name, detail) in enumerate(
            (
                ("manus", "MANUS 数据发布器", "/manus_glove_0 / /manus_glove_1"),
                ("ros", "ROS2 图连接", "节点与话题发现"),
            )
        ):
            box = card()
            layout = QVBoxLayout(box)
            layout.setContentsMargins(20, 18, 20, 18)
            head = QHBoxLayout()
            dot = status_dot(False)
            label = QLabel(name)
            label.setObjectName("cardTitle")
            head.addWidget(dot)
            head.addWidget(label)
            head.addStretch()
            value = QLabel("检查中")
            value.setObjectName("statusValue")
            layout.addLayout(head)
            layout.addWidget(QLabel(detail))
            layout.addWidget(value)
            grid.addWidget(box, index // 2, index % 2)
            self.rows[key] = (dot, value)
        root.addLayout(grid)

        self.summary = QLabel("正在检查本机环境…")
        self.summary.setObjectName("summary")
        root.addWidget(self.summary)
        root.addStretch()

    def refresh(self) -> None:
        if self._probe_active:
            return
        self._probe_active = True
        self.probe.start("ros2", ["topic", "list"])
        self.probe_timeout.start(1000)

    def _probe_finished(self, *_args) -> None:
        output = bytes(self.probe.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._complete_probe(set(output.splitlines()))

    def _probe_error(self, error) -> None:
        if error == QProcess.FailedToStart:
            self._complete_probe(set())

    def _probe_timed_out(self) -> None:
        if self.probe.state() != QProcess.NotRunning:
            self.probe.kill()
        else:
            self._complete_probe(set())

    def _complete_probe(self, topic_names: set[str]) -> None:
        if not self._probe_active:
            return
        self._probe_active = False
        self.probe_timeout.stop()
        self._apply_topics(topic_names)
        self.topics_ready.emit(topic_names)

    def _apply_topics(self, topic_names: set[str]) -> None:
        ros_ok = bool(topic_names)
        self._set("ros", ros_ok, "ROS2 在线" if ros_ok else "未发现 ROS2 图")
        manus_ok = "/manus_glove_0" in topic_names or "/manus_glove_1" in topic_names
        self._set("manus", manus_ok, "话题已发现" if manus_ok else "未发现手套话题")
        count = sum(dot.property("status") == "ok" for dot, _ in self.rows.values())
        self.summary.setText(f"{count}/{len(self.rows)} 项连接正常 · 最后检查：刚刚")

    def stop(self) -> None:
        self.probe_timeout.stop()
        if self.probe.state() != QProcess.NotRunning:
            self.probe.kill()
            self.probe.waitForFinished(100)

    def _set(self, key: str, ok: bool, text: str) -> None:
        dot, value = self.rows[key]
        dot.setProperty("status", "ok" if ok else "bad")
        dot.style().unpolish(dot)
        dot.style().polish(dot)
        value.setText(text)


class GestureIllustration(QWidget):
    """Small animated hand sketch; avoids adding an image asset dependency."""

    def __init__(self) -> None:
        super().__init__()
        self.gesture = "open"
        self.phase = 0.0
        self.setMinimumSize(150, 150)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(45)

    def set_gesture(self, gesture: str) -> None:
        self.gesture = gesture
        self.update()

    def _tick(self) -> None:
        self.phase += 0.16
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() * 0.5, self.height() * 0.54)
        pulse = 5.0 + 2.0 * (1.0 + sin(self.phase))
        painter.setPen(QPen(QColor("#9db7d7"), 2))
        painter.setBrush(QBrush(QColor("#e4eefb")))
        painter.drawEllipse(center, pulse, pulse)

        palm = QPainterPath()
        palm.moveTo(center.x() - 42, center.y() + 62)
        palm.cubicTo(center.x() - 54, center.y() + 18, center.x() - 48, center.y() - 16, center.x() - 35, center.y() - 24)
        palm.cubicTo(center.x() - 18, center.y() - 36, center.x() + 32, center.y() - 30, center.x() + 43, center.y() - 8)
        palm.cubicTo(center.x() + 52, center.y() + 18, center.x() + 45, center.y() + 48, center.x() + 34, center.y() + 62)
        palm.closeSubpath()
        painter.setPen(QPen(QColor("#2d405b"), 3))
        painter.setBrush(QBrush(QColor("#f9fbff")))
        painter.drawPath(palm)

        closed = self.gesture in {"fist", "index", "middle", "ring", "pinky"}
        finger_x = (-28, -10, 9, 27)
        finger_height = 25 if closed else 67
        for x in finger_x:
            base = QPointF(center.x() + x, center.y() - 9)
            tip = QPointF(center.x() + x, center.y() - finger_height)
            painter.drawLine(base, tip)
            painter.drawEllipse(tip, 6, 6)

        thumb_tip = QPointF(center.x() - 56, center.y() + 20)
        if self.gesture == "index":
            thumb_tip = QPointF(center.x() - 20, center.y() - 44)
        elif self.gesture == "middle":
            thumb_tip = QPointF(center.x() - 2, center.y() - 44)
        elif self.gesture == "ring":
            thumb_tip = QPointF(center.x() + 18, center.y() - 36)
        elif self.gesture == "pinky":
            thumb_tip = QPointF(center.x() + 34, center.y() - 24)
        painter.drawLine(QPointF(center.x() - 32, center.y() + 33), thumb_tip)
        painter.drawEllipse(thumb_tip, 7, 7)


class MoviePreview(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(140, 220)
        self._movie: QMovie | None = None

    def set_movie(self, path: Path) -> None:
        if self._movie is not None:
            self._movie.stop()
        self._movie = QMovie(str(path))
        self._movie.setCacheMode(QMovie.CacheAll)
        self._movie.frameChanged.connect(self._refresh_frame)
        self._movie.start()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._refresh_frame()

    def _refresh_frame(self, _frame: int = 0) -> None:
        if self._movie is not None:
            frame = self._movie.currentPixmap()
            if frame.isNull():
                return
            self.setPixmap(
                frame.scaled(
                    max(1, self.width()),
                    max(1, self.height()),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )


class SkeletonPreview(QWidget):
    HAND_NAMES = {"left": "左手", "right": "右手"}
    HAND_COLORS = {"left": QColor("#3b82f6"), "right": QColor("#16a36a")}

    def __init__(self) -> None:
        super().__init__()
        self.nodes: dict[str, dict[int, tuple[int, float, float, float]]] = {}
        self.selected_hand: str | None = None
        self.views = {
            hand: {"yaw": 0.78, "pitch": -0.28, "zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0}
            for hand in ("left", "right")
        }
        self.drag_hand: str | None = None
        self.drag_button = Qt.NoButton
        self.last_mouse = QPointF()
        self.setMinimumSize(220, 220)
        self.setMouseTracking(True)
        self.setToolTip("左键旋转，右键或中键平移，滚轮缩放，双击复位")

    def set_nodes(self, hand: str, nodes: dict[int, tuple[int, float, float, float]]) -> None:
        if hand in self.views:
            self.nodes[hand] = nodes
            self.update()

    def set_selected_hand(self, hand: str | None) -> None:
        self.selected_hand = hand if hand in self.views else None
        self.update()

    def visible_hands(self) -> list[str]:
        if self.selected_hand and self.nodes.get(self.selected_hand):
            return [self.selected_hand]
        return [hand for hand in ("left", "right") if self.nodes.get(hand)]

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f2f2f7"))
        hands = self.visible_hands()
        if not hands:
            painter.setPen(QColor(MUTED))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待手指骨架数据")
            return
        width = self.width() / len(hands)
        for index, hand in enumerate(hands):
            area = self.rect().adjusted(int(index * width) + 10, 10, int((index + 1) * width - self.width()) - 10, -10)
            self._paint_hand(painter, hand, area)
        if len(hands) == 2:
            painter.setPen(QPen(QColor("#d2d2d7"), 1))
            painter.drawLine(self.width() // 2, 12, self.width() // 2, self.height() - 12)

    def _paint_hand(self, painter: QPainter, hand: str, area) -> None:
        nodes = self.nodes[hand]
        points = [(node_id, parent, x, y, z) for node_id, (parent, x, y, z) in nodes.items()]
        if not points:
            return
        min_x, max_x = min(point[2] for point in points), max(point[2] for point in points)
        min_y, max_y = min(point[3] for point in points), max(point[3] for point in points)
        min_z, max_z = min(point[4] for point in points), max(point[4] for point in points)
        center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
        span = max(max_x - min_x, max_y - min_y, max_z - min_z, 1e-5)
        view = self.views[hand]
        scale = min(area.width(), area.height()) * 0.76 * view["zoom"] / span
        projected: dict[int, tuple[QPointF, float]] = {}
        cy, sy = cos(view["yaw"]), sin(view["yaw"])
        cp, sp = cos(view["pitch"]), sin(view["pitch"])
        for node_id, _parent, x, y, z in points:
            x, y, z = x - center[0], y - center[1], z - center[2]
            rotated_x = cy * x - sy * y
            rotated_y = sy * x + cy * y
            screen_y = cp * z - sp * rotated_y
            depth = sp * z + cp * rotated_y
            projected[node_id] = (
                QPointF(
                    area.center().x() + rotated_x * scale + view["pan_x"],
                    area.center().y() - screen_y * scale + view["pan_y"],
                ),
                depth,
            )

        color = self.HAND_COLORS[hand]
        bones = []
        for node_id, (parent, *_position) in nodes.items():
            if parent in projected and node_id in projected and parent != node_id:
                bones.append((projected[node_id][1] + projected[parent][1], parent, node_id))
        painter.setPen(QPen(color, 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for _depth, parent, node_id in sorted(bones):
            painter.drawLine(projected[parent][0], projected[node_id][0])
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(color)
        for point, _depth in projected.values():
            painter.drawEllipse(point, 5, 5)
        painter.setPen(QColor(TEXT))
        painter.drawText(area.adjusted(8, 6, -8, -6), Qt.AlignTop | Qt.AlignLeft, self.HAND_NAMES[hand])

    def _hand_at(self, x: float) -> str | None:
        hands = self.visible_hands()
        if not hands:
            return None
        if len(hands) == 1:
            return hands[0]
        return hands[0] if x < self.width() / 2 else hands[1]

    def mousePressEvent(self, event) -> None:
        self.drag_hand = self._hand_at(event.pos().x())
        self.drag_button = event.button()
        self.last_mouse = event.localPos()

    def mouseMoveEvent(self, event) -> None:
        if self.drag_hand is None or not (event.buttons() & self.drag_button):
            return
        delta = event.localPos() - self.last_mouse
        view = self.views[self.drag_hand]
        if self.drag_button == Qt.LeftButton:
            view["yaw"] += delta.x() * 0.01
            view["pitch"] = max(-1.45, min(1.45, view["pitch"] + delta.y() * 0.01))
        else:
            view["pan_x"] += delta.x()
            view["pan_y"] += delta.y()
        self.last_mouse = event.localPos()
        self.update()

    def mouseReleaseEvent(self, _event) -> None:
        self.drag_hand = None
        self.drag_button = Qt.NoButton

    def wheelEvent(self, event) -> None:
        hand = self._hand_at(event.pos().x())
        if hand is None:
            return
        factor = 1.12 if event.angleDelta().y() > 0 else 1 / 1.12
        self.views[hand]["zoom"] = max(0.1, min(2.5, self.views[hand]["zoom"] * factor))
        self.update()

    def mouseDoubleClickEvent(self, event) -> None:
        hand = self._hand_at(event.pos().x())
        if hand is not None:
            self.views[hand].update(yaw=0.78, pitch=-0.28, zoom=1.0, pan_x=0.0, pan_y=0.0)
            self.update()


class ManusGloveCalibrationPage(QWidget):
    GUIDES = (
        ("张开手掌", "前臂保持稳定，手掌自然摊平，五指放松分开，拇指朝外伸展。"),
        ("自然握拳", "四指自然卷曲成拳，拇指贴在食指外侧；不要用力挤压或扭转手腕。"),
        ("四指指尖触掌根", "保持手腕稳定，让四指指尖反复触碰靠近手腕的掌根位置。"),
        ("拇指触小指根", "让拇指反复触碰小指掌指关节附近，再向外伸展拇指。"),
    )
    FAMILY_NAMES = {
        4: "MetaGlove",
        7: "MetaGlove Pro",
        8: "MetaGlove Pro Precision",
        9: "MetaGlove Pro Haptics",
        10: "MetaGlove Pro Precision Haptics",
    }

    def __init__(self, *, auto_start_bridge: bool = True) -> None:
        super().__init__()
        self.bridge_ready = False
        self.available_hands: dict[str, tuple[int, int]] = {}
        self.selected_hand: str | None = None
        self.active = False
        self.busy = False
        self.step_complete = False
        self.current_step = 0
        self.step_count = 0
        self._stdout_buffer = ""
        self.notice_animation: QPropertyAnimation | None = None
        self.progress_animation: QPropertyAnimation | None = None
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self._read_bridge_output)
        self.process.errorOccurred.connect(self._bridge_error)
        self.process.finished.connect(self._bridge_finished)

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 28)
        root.setSpacing(16)
        top = QHBoxLayout()
        title = QLabel("MANUS 手套标定")
        title.setObjectName("pageTitle")
        top.addWidget(title)
        top.addStretch()
        top.addWidget(QLabel("标定手"))
        selector = QFrame()
        selector.setObjectName("handSelector")
        selector_layout = QHBoxLayout(selector)
        selector_layout.setContentsMargins(3, 3, 3, 3)
        selector_layout.setSpacing(2)
        self.hand_buttons: dict[str, QPushButton] = {}
        for hand, text in (("left", "左手"), ("right", "右手")):
            button = QPushButton(text)
            button.setObjectName("handChoice")
            button.setCheckable(True)
            button.clicked.connect(lambda _checked, value=hand: self._choose_hand(value))
            selector_layout.addWidget(button)
            self.hand_buttons[hand] = button
        top.addWidget(selector)
        self.hand_status = QLabel("连接中")
        self.hand_status.setObjectName("handStatus")
        top.addWidget(self.hand_status)
        root.addLayout(top)

        self.banner = QLabel("正在连接 MANUS 标定服务…")
        self.banner.setObjectName("notice")
        root.addWidget(self.banner)

        body = QHBoxLayout()
        body.setSpacing(18)
        self.step_list = QListWidget()
        self.step_list.setObjectName("stepList")
        self.step_list.setMinimumWidth(190)
        self.step_list.setMaximumWidth(230)
        self.step_list.setFocusPolicy(Qt.NoFocus)
        self.step_list.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.step_list.addItem("等待 MANUS SDK")
        body.addWidget(self.step_list)

        current = card()
        current_layout = QHBoxLayout(current)
        current_layout.setContentsMargins(24, 24, 24, 24)
        current_layout.setSpacing(24)
        preview = QFrame()
        preview.setObjectName("preview")
        preview_layout = QVBoxLayout(preview)
        self.preview_stack = QStackedWidget()
        self.skeleton = SkeletonPreview()
        self.movie = MoviePreview()
        self.preview_stack.addWidget(self.skeleton)
        self.preview_stack.addWidget(self.movie)
        self.preview_stack.setCurrentWidget(self.skeleton)
        preview_layout.addWidget(self.preview_stack, 1)
        self.preview_text = QLabel("连接手套后开始")
        self.preview_text.setObjectName("previewText")
        self.preview_text.setAlignment(Qt.AlignCenter)
        preview_layout.addWidget(self.preview_text)
        current_layout.addWidget(preview, 1)

        controls = QVBoxLayout()
        self.step_title = QLabel("准备标定")
        self.step_title.setObjectName("sectionTitle")
        controls.addWidget(self.step_title)
        self.step_hint = QLabel("Qt 将按 MANUS SDK 返回的实际步骤数执行标定。")
        self.step_hint.setObjectName("subtitle")
        self.step_hint.setWordWrap(True)
        controls.addWidget(self.step_hint)
        self.device = QLabel("尚未选择手套")
        self.device.setObjectName("result")
        controls.addWidget(self.device)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        controls.addWidget(self.progress_bar)
        self.action = QPushButton("开始标定")
        self.action.setObjectName("primary")
        self.action.clicked.connect(self._primary_action)
        controls.addWidget(self.action)
        self.next_button = QPushButton("下一步")
        self.next_button.clicked.connect(lambda: self._send("next"))
        controls.addWidget(self.next_button)
        self.restart_button = QPushButton("重新标定")
        self.restart_button.clicked.connect(self.restart)
        controls.addWidget(self.restart_button)
        controls.addStretch()
        current_layout.addLayout(controls, 1)
        body.addWidget(current, 1)
        root.addLayout(body, 1)

        result_box = card()
        result_layout = QVBoxLayout(result_box)
        result_layout.addWidget(QLabel("标定结果"))
        self.result = QLabel("尚未生成 .mcal 文件")
        self.result.setObjectName("result")
        self.result.setWordWrap(True)
        result_layout.addWidget(self.result)
        root.addWidget(result_box)
        self._sync_controls()
        if auto_start_bridge:
            QTimer.singleShot(0, self.start_bridge)

    def start_bridge(self) -> None:
        if self.process.state() != QProcess.NotRunning:
            return
        self.process.start("/usr/bin/python3", [str(BRIDGE_SCRIPT)])

    def _read_bridge_output(self) -> None:
        self._stdout_buffer += bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        while "\n" in self._stdout_buffer:
            line, self._stdout_buffer = self._stdout_buffer.split("\n", 1)
            if line:
                self.handle_status(line.rstrip("\r"))

    def handle_status(self, line: str) -> None:
        fields = line.split("\t")
        event = fields[0]
        if event == "BRIDGE_READY":
            self.bridge_ready = True
            self._send("status")
        elif event == "READY":
            self.banner.setText("MANUS 标定服务已连接，请选择手套")
            self._set_notice("info")
        elif event == "GLOVES" and len(fields) >= 5:
            hands: dict[str, tuple[int, int]] = {}
            for hand, id_text, family_text in (("left", fields[1], fields[3]), ("right", fields[2], fields[4])):
                glove_id = int(id_text)
                if glove_id:
                    hands[hand] = (glove_id, int(family_text))
            self._update_gloves(hands)
        elif event == "SKELETON" and len(fields) >= 3 and fields[1] in {"left", "right"}:
            self._update_skeleton(fields[1], fields[2])
            return
        elif event == "STARTED" and len(fields) >= 5:
            self.active = True
            self.busy = False
            self.step_complete = False
            self.current_step = 0
            self.step_count = int(fields[3])
            self._populate_steps()
            family = self.FAMILY_NAMES.get(int(fields[4]), f"设备族 {fields[4]}")
            self.device.setText(f"{family} · ID {fields[2]} · SDK 返回 {self.step_count} 步")
        elif event == "STEP_READY" and len(fields) >= 6:
            self.active = True
            self.busy = False
            self.step_complete = False
            self.current_step = int(fields[1])
            self.step_count = int(fields[2])
            self._show_step(fields[3], fields[4])
            self.result.setText("当前姿势准备好后，点击“采集当前步骤”。")
        elif event == "CAPTURING":
            self.busy = True
            self.banner.setText("MANUS SDK 正在计算当前步骤，请按图示持续完成动作…")
            self._set_notice("info")
        elif event == "STEP_DONE":
            self.busy = False
            self.step_complete = True
            self.result.setText(f"第 {self.current_step + 1} 步已完成，可以进入下一步。")
            self.banner.setText("当前步骤已完成")
            self._set_notice("success")
        elif event == "FINISHED" and len(fields) >= 2:
            self.active = False
            self.busy = False
            self.step_complete = False
            self._animate_progress(100)
            self.result.setText(f"MANUS 手套标定已保存：\n{fields[1]}")
            self.banner.setText("MANUS 手套标定完成")
            self._set_notice("success")
            self.preview_stack.setCurrentWidget(self.skeleton)
        elif event == "STOPPED":
            self._reset_state("标定已重置，请重新开始。")
        elif event == "ERROR":
            self.busy = False
            self.result.setText(f"MANUS 手套标定失败：{fields[1] if len(fields) > 1 else '未知错误'}")
            self.banner.setText("标定操作未完成，请检查连接后重试")
            self._set_notice("warning")
        self._sync_controls()

    def _update_skeleton(self, hand: str, payload: str) -> None:
        nodes: dict[int, tuple[int, float, float, float]] = {}
        try:
            for encoded in payload.split(";")[:128]:
                if not encoded:
                    continue
                node_text, parent_text, x_text, y_text, z_text = encoded.split(",")
                position = (float(x_text), float(y_text), float(z_text))
                if all(isfinite(value) for value in position):
                    nodes[int(node_text)] = (int(parent_text), *position)
        except ValueError:
            return
        self.skeleton.set_nodes(hand, nodes)
        if not self.active:
            self.preview_text.setText("实时手指骨架")

    def _update_gloves(self, hands: dict[str, tuple[int, int]]) -> None:
        self.available_hands = hands
        if not self.active:
            if len(hands) == 1:
                self.selected_hand = next(iter(hands))
            elif self.selected_hand not in hands:
                self.selected_hand = None
        if not hands:
            self.banner.setText("未检测到 MANUS 手套，请检查 Dongle 和手套电源")
            self._set_notice("warning")
        elif len(hands) == 2:
            self.banner.setText("左右手套均已连接，请选择本次标定手")
            self._set_notice("info")
        else:
            name = "左手" if "left" in hands else "右手"
            self.banner.setText(f"已自动识别{name}手套，可以开始标定")
            self._set_notice("success")
        self.skeleton.set_selected_hand(self.selected_hand)
        self._sync_controls()

    def _choose_hand(self, hand: str) -> None:
        if not self.active and not self.busy and hand in self.available_hands:
            self.selected_hand = hand
            self.skeleton.set_selected_hand(hand)
        self._sync_controls()

    def _primary_action(self) -> None:
        if not self.active:
            if self.selected_hand:
                self.busy = True
                self.result.setText("正在向 MANUS SDK 启动标定…")
                self._send(f"start\t{self.selected_hand}")
        elif not self.step_complete:
            self.busy = True
            self._send("capture")
        self._sync_controls()

    def restart(self) -> None:
        if self.active:
            self.busy = True
            self._send("stop")
        else:
            self._reset_state("请重新选择标定手并开始。")
        self._sync_controls()

    def _send(self, command: str) -> None:
        if self.process.state() == QProcess.Running:
            self.process.write((command + "\n").encode("utf-8"))

    def _populate_steps(self) -> None:
        self.step_list.clear()
        for index in range(self.step_count):
            title = self.GUIDES[index][0] if index < len(self.GUIDES) else "SDK 标定步骤"
            self.step_list.addItem(f"{index + 1:02d}   {title}")

    def _show_step(self, sdk_title: str, sdk_description: str) -> None:
        guide_title, guide_hint = self.GUIDES[self.current_step] if self.current_step < len(self.GUIDES) else (sdk_title, sdk_description)
        self.step_title.setText(f"{self.current_step + 1}. {guide_title}")
        self.step_hint.setText(guide_hint or sdk_description)
        self.preview_text.setText("按图示完成动作")
        self.step_list.setCurrentRow(self.current_step)
        self._animate_progress(int(100 * self.current_step / max(1, self.step_count)))
        movie_index = min(self.current_step + 1, 4)
        self.movie.set_movie(ASSET_DIR / "manus" / f"MANUS_Core_Calibration_{movie_index:02d}.gif")
        self.preview_stack.setCurrentWidget(self.movie)

    def _reset_state(self, text: str) -> None:
        self.active = False
        self.busy = False
        self.step_complete = False
        self.current_step = 0
        self.step_count = 0
        self.progress_bar.setValue(0)
        self.step_list.clear()
        self.step_list.addItem("等待开始")
        self.step_title.setText("准备标定")
        self.step_hint.setText("Qt 将按 MANUS SDK 返回的实际步骤数执行标定。")
        self.result.setText(text)
        self.preview_text.setText("实时手指骨架")
        self.preview_stack.setCurrentWidget(self.skeleton)

    def _set_notice(self, state: str) -> None:
        self.banner.setProperty("notice", state)
        self.banner.style().unpolish(self.banner)
        self.banner.style().polish(self.banner)
        effect = QGraphicsOpacityEffect(self.banner)
        self.banner.setGraphicsEffect(effect)
        animation = QPropertyAnimation(effect, b"opacity", self)
        animation.setDuration(180)
        animation.setStartValue(0.55)
        animation.setEndValue(1.0)
        animation.setEasingCurve(QEasingCurve.OutCubic)
        animation.start()
        self.notice_animation = animation

    def _animate_progress(self, value: int) -> None:
        if self.progress_animation is not None:
            self.progress_animation.stop()
        animation = QPropertyAnimation(self.progress_bar, b"value", self)
        animation.setDuration(260)
        animation.setStartValue(self.progress_bar.value())
        animation.setEndValue(value)
        animation.setEasingCurve(QEasingCurve.OutCubic)
        animation.start()
        self.progress_animation = animation

    def _sync_controls(self) -> None:
        selectable = not self.active and not self.busy and len(self.available_hands) == 2
        for hand, button in self.hand_buttons.items():
            button.setChecked(hand == self.selected_hand)
            button.setEnabled(selectable and hand in self.available_hands)
        if self.active:
            self.hand_status.setText("标定中")
        elif not self.available_hands:
            self.hand_status.setText("未检测到")
        elif len(self.available_hands) == 1:
            self.hand_status.setText("已自动识别")
        elif self.selected_hand is None:
            self.hand_status.setText("请选择")
        else:
            self.hand_status.setText("双手在线")
        self.action.setText("采集当前步骤" if self.active else "开始标定")
        self.action.setEnabled(
            self.bridge_ready and not self.busy and self.selected_hand in self.available_hands and not self.step_complete
        )
        self.next_button.setText("完成并保存" if self.step_count and self.current_step + 1 == self.step_count else "下一步")
        self.next_button.setEnabled(self.active and self.step_complete and not self.busy)
        self.restart_button.setEnabled(not self.busy and (self.active or self.step_count > 0))

    def _bridge_error(self, _error) -> None:
        self.bridge_ready = False
        self.banner.setText("无法启动 MANUS 标定桥接服务")
        self._set_notice("warning")
        self._sync_controls()

    def _bridge_finished(self) -> None:
        self.bridge_ready = False
        self._sync_controls()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.step_list.setVisible(self.width() >= 720)

    def stop(self) -> None:
        if self.active:
            self._send("stop")
            self.process.waitForBytesWritten(300)
        if self.process.state() != QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(1000):
                self.process.kill()
                self.process.waitForFinished(500)


class ManusL20CalibrationPage(QWidget):
    GLOVE_TOPICS = {"right": "/manus_glove_0", "left": "/manus_glove_1"}
    STEPS = (
        ("选择设备", "确认标定手和输入话题，然后初始化 ROS2 标定会话"),
        ("自然张开", "手掌自然张开，保持 2 秒"),
        ("四指握拳", "四指完全弯曲，拇指保持放松"),
        ("四指并拢", "四指自然并拢，采集 yaw close"),
        ("四指外展", "四指充分外展，采集 yaw spread"),
        ("拇指触碰小指根", "用于拇指弯曲和 segment IK touch frame"),
        ("指尖接触", "依次采集自然张开和四组拇指指尖接触"),
        ("标定完成", "检查本次生成的标定文件"),
    )
    FULL_POSES = {
        1: "natural_open",
        2: "four_finger_fist",
        3: "finger_close",
        4: "finger_spread",
        5: "thumb_pinky_root_touch",
    }
    STEP_IMAGES = {
        0: "05-自然张开.png",
        1: "05-自然张开.png",
        2: "03-握拳.png",
        3: "02-闭合.png",
        4: "01-外展.png",
        5: "04-大拇指触小拇指根.png",
        7: "05-自然张开.png",
    }
    CONTACT_POSES = (
        "natural_open",
        "thumb_index_tip_touch",
        "thumb_middle_tip_touch",
        "thumb_ring_tip_touch",
        "thumb_pinky_tip_touch",
    )
    CONTACT_GESTURES = ("open", "index", "middle", "ring", "pinky")
    CONTACT_NAMES = ("自然张开", "拇指 · 食指指尖", "拇指 · 中指指尖", "拇指 · 无名指指尖", "拇指 · 小指指尖")
    CONTACT_IMAGES = (
        "05-自然张开.png",
        "06-食指对指.png",
        "07-中指对指.png",
        "08-无名指对指.png",
        "09-小拇指对指.png",
    )

    def __init__(
        self,
        *,
        full_factory: Callable[..., Any] = full_session_factory,
        contact_factory: Callable[..., Any] = contact_session_factory,
        task_runner: Callable[[Callable[[], Any], Callable[[Any], None], Callable[[str], None]], None] | None = None,
    ) -> None:
        super().__init__()
        self.step = 0
        self.progress = 0
        self.full_factory = full_factory
        self.contact_factory = contact_factory
        self.task_runner = task_runner
        self.full_session: Any | None = None
        self.contact_session: Any | None = None
        self.contact_index = 0
        self.saved_paths: list[str] = []
        self.available_hands: set[str] = set()
        self.selected_hand: str | None = None
        self.glove_detection_initialized = False
        self.glove_notice = ("info", "标定只订阅 MANUS 数据，不会向 L20 发送运动命令")
        self.calibration_started = False
        self.busy = False
        self.generation = 0
        self.task: TaskThread | None = None
        self.progress_animation: QPropertyAnimation | None = None
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 24, 28, 28)
        root.setSpacing(16)

        top = QHBoxLayout()
        title = QLabel("MANUS-L20 标定")
        title.setObjectName("pageTitle")
        top.addWidget(title)
        top.addStretch()
        top.addWidget(QLabel("标定手"))
        self.hand_selector = QFrame()
        self.hand_selector.setObjectName("handSelector")
        hand_layout = QHBoxLayout(self.hand_selector)
        hand_layout.setContentsMargins(3, 3, 3, 3)
        hand_layout.setSpacing(2)
        self.hand_buttons: dict[str, QPushButton] = {}
        for hand, text in (("left", "左手"), ("right", "右手")):
            button = QPushButton(text)
            button.setObjectName("handChoice")
            button.setCheckable(True)
            button.clicked.connect(lambda _checked, value=hand: self._choose_hand(value))
            hand_layout.addWidget(button)
            self.hand_buttons[hand] = button
        top.addWidget(self.hand_selector)
        self.hand_status = QLabel("识别中")
        self.hand_status.setObjectName("handStatus")
        top.addWidget(self.hand_status)
        root.addLayout(top)

        self.banner = QLabel("标定只订阅 MANUS 数据，不会向 L20 发送运动命令")
        self.banner.setObjectName("notice")
        self._render_glove_notice()
        root.addWidget(self.banner)

        body = QHBoxLayout()
        body.setSpacing(18)
        self.step_list = QListWidget()
        self.step_list.setObjectName("stepList")
        self.step_list.setMinimumWidth(180)
        self.step_list.setMaximumWidth(210)
        self.step_list.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.step_list.setWordWrap(True)
        self.step_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.step_list.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.step_list.setFocusPolicy(Qt.NoFocus)
        self.step_list.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        for index, (name, hint) in enumerate(self.STEPS):
            item = QListWidgetItem(f"{index + 1:02d}   {name}")
            item.setSizeHint(QSize(220, 50))
            self.step_list.addItem(item)
        self.step_list.setCurrentRow(0)
        self.step_list.currentRowChanged.connect(self._keep_current_step_selected)
        body.addWidget(self.step_list)

        content = QVBoxLayout()
        current = card()
        self.current_layout = QHBoxLayout(current)
        self.current_layout.setContentsMargins(24, 24, 24, 24)
        preview = QFrame()
        preview.setObjectName("preview")
        preview_layout = QVBoxLayout(preview)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(0)
        self.preview_stack = QStackedWidget()
        self.preview_stack.setMinimumSize(150, 150)
        self.preview_stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.gesture_image = GestureImage()
        self.illustration = GestureIllustration()
        self.preview_stack.addWidget(self.gesture_image)
        self.preview_stack.addWidget(self.illustration)
        preview_layout.addWidget(self.preview_stack, 1)
        self.preview_text = QLabel("准备开始")
        self.preview_text.setAlignment(Qt.AlignCenter)
        self.preview_text.setObjectName("previewText")
        preview_layout.addWidget(self.preview_text)
        self.current_layout.addWidget(preview, 1)

        controls = QVBoxLayout()
        self.step_title = QLabel()
        self.step_title.setObjectName("sectionTitle")
        controls.addWidget(self.step_title)
        self.step_hint = QLabel()
        self.step_hint.setWordWrap(True)
        self.step_hint.setObjectName("subtitle")
        controls.addWidget(self.step_hint)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(18)
        controls.addWidget(self.progress_bar)
        self.action = QPushButton("开始采集")
        self.action.setObjectName("primary")
        self.action.clicked.connect(self.capture)
        controls.addWidget(self.action)
        self.next_button = QPushButton("下一步")
        self.next_button.setEnabled(False)
        self.next_button.clicked.connect(self.next_step)
        controls.addWidget(self.next_button)
        self.back_button = QPushButton("上一步")
        self.back_button.clicked.connect(self.previous_step)
        controls.addWidget(self.back_button)
        self.restart_button = QPushButton("重新标定")
        self.restart_button.clicked.connect(self.restart_calibration)
        controls.addWidget(self.restart_button)
        controls.addStretch()
        self.current_layout.addLayout(controls, 1)
        content.addWidget(current)

        result_box = card()
        result_layout = QVBoxLayout(result_box)
        result_layout.setContentsMargins(20, 16, 20, 16)
        result_layout.addWidget(QLabel("标定结果"), alignment=Qt.AlignLeft)
        self.result = QLabel("尚未采集数据")
        self.result.setObjectName("result")
        self.result.setWordWrap(True)
        result_layout.addWidget(self.result)
        content.addWidget(result_box)
        body.addLayout(content, 1)
        root.addLayout(body, 1)
        self.update_step()

    def update_step(self) -> None:
        name, hint = self.STEPS[self.step]
        self.step_title.setText(f"{self.step + 1}. {name}")
        self.step_hint.setText(hint)
        self._update_preview()
        self.preview_text.setText("准备开始" if self.progress == 0 else "已完成，可进入下一步")
        self.action.setText("重新采集" if self.progress else "采集当前姿势")
        if self.step == 0:
            self.action.setText("开始标定")
        elif self.step == 6:
            if self.contact_index < len(self.CONTACT_NAMES):
                self.preview_text.setText(self.CONTACT_NAMES[self.contact_index])
                self.action.setText("采集当前姿势")
            else:
                self.preview_text.setText("指尖接触标定已完成")
                self.action.setText("已完成")
        elif self.step == len(self.STEPS) - 1:
            self.action.setText("完成")
        self.action.setEnabled(
            not self.busy
            and self.selected_hand in self.available_hands
            and not (self.step == 6 and self.contact_index >= len(self.CONTACT_NAMES))
        )
        self.next_button.setEnabled(
            not self.busy and self.progress == 100 and self.step < len(self.STEPS) - 1
        )
        self.back_button.setEnabled(not self.busy and self.step > 0)
        self.restart_button.setEnabled(
            not self.busy and (self.calibration_started or self.step > 0 or self.progress > 0)
        )
        self._sync_hand_selector()
        self.step_list.setCurrentRow(self.step)

    def update_glove_topics(self, topics: set[str]) -> None:
        previous = self.available_hands
        available = {hand for hand, topic in self.GLOVE_TOPICS.items() if topic in topics}
        self.available_hands = available
        if not self.calibration_started:
            if len(available) == 1:
                self.selected_hand = next(iter(available))
            elif not available or self.selected_hand not in available:
                self.selected_hand = None
        added = available - previous
        removed = previous - available
        if removed:
            self._set_glove_notice(
                "warning",
                f"{self._hand_names(removed)}手套连接已断开，请重新连接后继续",
            )
        elif added or not self.glove_detection_initialized:
            if not available:
                self._set_glove_notice("warning", "未检测到 MANUS 手套，请连接手套并等待自动识别")
            elif len(available) == 2:
                self._set_glove_notice("info", "已检测到左右手手套，请在右上角选择标定手")
            else:
                self._set_glove_notice(
                    "success",
                    f"已自动识别{self._hand_names(available)}手套，可以开始标定",
                )
        self.glove_detection_initialized = True
        self.update_step()

    @staticmethod
    def _hand_names(hands: set[str]) -> str:
        return "、".join(name for hand, name in (("left", "左手"), ("right", "右手")) if hand in hands)

    def _set_glove_notice(self, state: str, text: str) -> None:
        self.glove_notice = (state, text)
        self._render_glove_notice()

    def _render_glove_notice(self) -> None:
        state, text = self.glove_notice
        self.banner.setText(text)
        self.banner.setProperty("notice", state)
        self.banner.style().unpolish(self.banner)
        self.banner.style().polish(self.banner)

    def _choose_hand(self, hand: str) -> None:
        if not self.calibration_started and not self.busy and len(self.available_hands) == 2:
            self.selected_hand = hand
        self.update_step()

    def _sync_hand_selector(self) -> None:
        selectable = not self.calibration_started and not self.busy and len(self.available_hands) == 2
        for hand, button in self.hand_buttons.items():
            button.setChecked(hand == self.selected_hand)
            button.setEnabled(selectable)
        if self.calibration_started and self.selected_hand not in self.available_hands:
            self.hand_status.setText("连接中断")
        elif self.calibration_started:
            self.hand_status.setText("标定中")
        elif not self.available_hands:
            self.hand_status.setText("未检测到")
        elif len(self.available_hands) == 1:
            self.hand_status.setText("已自动识别")
        elif self.selected_hand is None:
            self.hand_status.setText("请选择")
        else:
            self.hand_status.setText("双手在线")

    def _keep_current_step_selected(self, row: int) -> None:
        if row != self.step:
            self.step_list.setCurrentRow(self.step)

    def _update_preview(self) -> None:
        filename = self.STEP_IMAGES.get(self.step)
        if self.step == 6 and self.contact_index < len(self.CONTACT_IMAGES):
            filename = self.CONTACT_IMAGES[self.contact_index]
        if filename and self.gesture_image.set_image(ASSET_DIR / filename):
            self.preview_stack.setCurrentWidget(self.gesture_image)
            return
        gesture = "pinky" if self.step == 5 else "open"
        if self.step == 6 and self.contact_index < len(self.CONTACT_GESTURES):
            gesture = self.CONTACT_GESTURES[self.contact_index]
        self.illustration.set_gesture(gesture)
        self.preview_stack.setCurrentWidget(self.illustration)

    def capture(self) -> None:
        if self.busy:
            return
        if self.step == 0:
            self._start_calibration()
        elif self.step in self.FULL_POSES:
            self._capture_full_pose()
        elif self.step == 6:
            self._capture_contact_pose()
        else:
            QMessageBox.information(self, "标定完成", "标定文件已经写入配置目录。")

    def _session_options(self) -> dict[str, Any]:
        if self.selected_hand is None:
            raise RuntimeError("未检测到可用于标定的 MANUS 手套")
        hand = self.selected_hand
        return {
            "hand": hand,
            "glove_topic": self.GLOVE_TOPICS[hand],
            "duration": 2.0,
        }

    def _start_calibration(self) -> None:
        def operation():
            session = self.full_factory(**self._session_options())
            session.start()
            return session

        def succeeded(session) -> None:
            self.full_session = session
            self.calibration_started = True
            self._complete_step("ROS2 标定会话已启动，请进入自然张开姿势。")

        self._run_task(operation, succeeded)

    def _capture_full_pose(self) -> None:
        if self.full_session is None:
            self._fail_task("标定会话尚未启动，请返回第一步重新开始。")
            return
        label = self.FULL_POSES[self.step]
        final_pose = self.step == max(self.FULL_POSES)

        def operation():
            self.full_session.capture_pose(label)
            if not final_pose:
                return []
            try:
                return self.full_session.save()
            finally:
                self.full_session.close()

        def succeeded(paths) -> None:
            if paths:
                for path in paths:
                    if str(path) not in self.saved_paths:
                        self.saved_paths.append(str(path))
                detail = "基础标定完成，已保存 4 个配置文件。"
            else:
                detail = f"{self.STEPS[self.step][0]}采集完成。"
            self._complete_step(detail)

        self._run_task(operation, succeeded)

    def _capture_contact_pose(self) -> None:
        if self.contact_index >= len(self.CONTACT_POSES):
            return
        label = self.CONTACT_POSES[self.contact_index]
        final_pose = self.contact_index == len(self.CONTACT_POSES) - 1

        def operation():
            session = self.contact_session
            created = session is None
            if created:
                session = self.contact_factory(**self._session_options())
            try:
                session.capture_pose(label)
                path = session.save() if final_pose else None
                if final_pose:
                    session.close()
                return session, path
            except Exception:
                if created:
                    session.close()
                raise

        def succeeded(payload) -> None:
            session, path = payload
            self.contact_session = session
            self.contact_index += 1
            if path is not None:
                if str(path) not in self.saved_paths:
                    self.saved_paths.append(str(path))
                self.progress = 100
                self.result.setText("指尖接触标定完成，配置已保存；runtime.enabled 保持 false。")
            else:
                self.progress = int(100 * self.contact_index / len(self.CONTACT_POSES))
                self.result.setText(
                    f"已采集 {self.CONTACT_NAMES[self.contact_index - 1]}，请准备下一姿势。"
                )
            self._finish_busy()

        self._run_task(operation, succeeded)

    def _run_task(self, operation: Callable[[], Any], succeeded: Callable[[Any], None]) -> None:
        self.busy = True
        generation = self.generation
        self.banner.setText("正在采集 MANUS 数据，请保持当前姿势…")
        self.banner.setProperty("notice", "info")
        self.banner.style().unpolish(self.banner)
        self.banner.style().polish(self.banner)
        self.update_step()

        def current_success(value: Any) -> None:
            if generation == self.generation:
                succeeded(value)

        def current_failure(error: str) -> None:
            if generation == self.generation:
                self._fail_task(error)

        if self.task_runner is not None:
            self.task_runner(operation, current_success, current_failure)
            return
        task = TaskThread(operation, self)
        self.task = task
        task.succeeded.connect(current_success)
        task.failed.connect(current_failure)
        task.finished.connect(self._task_finished)
        task.start()

    def _task_finished(self) -> None:
        if self.task is not None:
            self.task.deleteLater()
        self.task = None

    def _complete_step(self, text: str) -> None:
        self.progress = 100
        self.result.setText(text)
        self.progress_animation = QPropertyAnimation(self.progress_bar, b"value", self)
        self.progress_animation.setDuration(450)
        self.progress_animation.setStartValue(self.progress_bar.value())
        self.progress_animation.setEndValue(100)
        self.progress_animation.start()
        self._finish_busy()

    def _fail_task(self, error: str) -> None:
        self.progress = 0
        self.result.setText(f"标定失败：{error}")
        self._finish_busy()

    def _finish_busy(self) -> None:
        self.busy = False
        self._render_glove_notice()
        self.progress_bar.setValue(self.progress)
        self.update_step()

    def previous_step(self) -> None:
        if self.busy or self.step <= 0:
            return
        if self.step == 1:
            self.restart_calibration()
            self.result.setText("已回退到：选择设备，请重新开始标定。")
            return
        if self.step >= 6:
            if self.contact_session is not None:
                self.contact_session.close()
            self.contact_session = None
            self.contact_index = 0
            self.saved_paths = self.saved_paths[:4]
        self.step -= 1
        self.progress = 0
        self.progress_bar.setValue(0)
        self.result.setText(f"已回退到：{self.STEPS[self.step][0]}，请重新采集当前姿势。")
        self.update_step()

    def restart_calibration(self) -> None:
        self.generation += 1
        self.stop_capture()
        self.full_session = None
        self.contact_session = None
        self.contact_index = 0
        self.saved_paths = []
        self.calibration_started = False
        self.busy = False
        self.step = 0
        self.progress = 0
        if self.progress_animation is not None:
            self.progress_animation.stop()
        self.progress_bar.setValue(0)
        self.result.setText("标定流程已重置，请重新选择标定手并开始。")
        self.update_step()

    def stop_capture(self) -> None:
        if self.task is not None and self.task.isRunning():
            self.task.wait(3500)
        if self.full_session is not None:
            self.full_session.close()
        if self.contact_session is not None:
            self.contact_session.close()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        direction = QBoxLayout.TopToBottom if self.width() < 620 else QBoxLayout.LeftToRight
        self.current_layout.setDirection(direction)

    def next_step(self) -> None:
        if self.busy or self.progress != 100 or self.step >= len(self.STEPS) - 1:
            return
        self.step += 1
        self.progress = 0
        self.progress_bar.setValue(0)
        self.update_step()
        if self.step == len(self.STEPS) - 1:
            files = "\n".join(self.saved_paths) if self.saved_paths else "未生成文件"
            self.result.setText(f"本次标定已完成：\n{files}")


CalibrationPage = ManusL20CalibrationPage


class MainWindow(QMainWindow):
    def __init__(self, *, auto_start_bridge: bool = True) -> None:
        super().__init__()
        self.setWindowTitle("灵巧手标定")
        self.resize(1360, 840)
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(236)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(22, 28, 22, 22)
        brand = QLabel("灵巧手标定")
        brand.setObjectName("brand")
        side.addWidget(brand)
        product = QLabel("MANUS · L20")
        product.setObjectName("product")
        side.addWidget(product, alignment=Qt.AlignLeft)
        side.addSpacing(34)
        nav = QListWidget()
        nav.setObjectName("nav")
        nav.addItem("MANUS 手套标定")
        nav.addItem("MANUS-L20 标定")
        nav.setCurrentRow(0)
        side.addWidget(nav)
        side.addStretch()
        footer = QLabel("本地标定工具")
        footer.setObjectName("sidebarFooter")
        side.addWidget(footer)
        layout.addWidget(sidebar)

        main = QVBoxLayout()
        top = QFrame()
        top.setObjectName("topbar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(28, 16, 32, 16)
        top_layout.addWidget(QLabel("设备与标定"))
        top_layout.addStretch()
        self.connection = QLabel("● 检查连接中")
        self.connection.setObjectName("connection")
        top_layout.addWidget(self.connection)
        main.addWidget(top)
        self.pages = QStackedWidget()
        glove_calibration = ManusGloveCalibrationPage(auto_start_bridge=auto_start_bridge)
        calibration = ManusL20CalibrationPage()
        self.pages.addWidget(glove_calibration)
        self.pages.addWidget(calibration)
        nav.currentRowChanged.connect(self._show_page)
        main.addWidget(self.pages)
        layout.addLayout(main, 1)
        self.setCentralWidget(root)

        self.system = SystemPage()
        self.system.topics_ready.connect(self._apply_system_topics)
        self.glove_calibration = glove_calibration
        self.calibration = calibration
        self.page_animation: QPropertyAnimation | None = None
        connection_effect = QGraphicsOpacityEffect(self.connection)
        self.connection.setGraphicsEffect(connection_effect)
        self.connection_animation = QPropertyAnimation(connection_effect, b"opacity", self)
        self.connection_animation.setDuration(1800)
        self.connection_animation.setKeyValueAt(0.0, 1.0)
        self.connection_animation.setKeyValueAt(0.5, 0.62)
        self.connection_animation.setKeyValueAt(1.0, 1.0)
        self.connection_animation.setEasingCurve(QEasingCurve.InOutSine)
        self.connection_animation.setLoopCount(-1)
        self.connection_animation.start()
        timer = QTimer(self)
        timer.timeout.connect(self.refresh_system)
        timer.start(3000)
        self.refresh_system()

    def _show_page(self, index: int) -> None:
        if self.page_animation is not None:
            previous_page = self.page_animation.targetObject()
            previous_end = self.page_animation.endValue()
            self.page_animation.stop()
            if previous_page is not None and isinstance(previous_end, QPoint):
                previous_page.move(previous_end)
        self.pages.setCurrentIndex(index)
        page = self.pages.currentWidget()
        end = page.pos()
        start = end + QPoint(14, 0)
        page.move(start)
        animation = QPropertyAnimation(page, b"pos", self)
        animation.setDuration(180)
        animation.setStartValue(start)
        animation.setEndValue(end)
        animation.setEasingCurve(QEasingCurve.OutCubic)
        animation.start()
        self.page_animation = animation

    def closeEvent(self, event) -> None:
        self.system.stop()
        self.glove_calibration.stop()
        self.calibration.stop_capture()
        event.accept()

    def refresh_system(self) -> None:
        self.system.refresh()

    def _apply_system_topics(self, topics: set[str]) -> None:
        self.calibration.update_glove_topics(topics)
        healthy = all(dot.property("status") == "ok" for dot, _ in self.system.rows.values())
        self.connection.setText("● 系统连接正常" if healthy else "● 有连接项待检查")
        self.connection.setProperty("status", "ok" if healthy else "bad")
        self.connection.style().unpolish(self.connection)
        self.connection.style().polish(self.connection)


def main() -> int:
    app = QApplication(sys.argv)
    apply_style(app)
    window = MainWindow()
    window.show()
    return app.exec_()


def apply_style(app: QApplication) -> None:
    app.setStyleSheet(
        f"""
        QWidget {{ background: {BG}; color: {TEXT}; font-family: 'SF Pro Display', 'SF Pro Text', 'Noto Sans CJK SC', sans-serif; font-size: 14px; }}
        QLabel {{ background: transparent; }}
        #sidebar {{ background: #fbfbfd; border-right: 1px solid #e5e5ea; }}
        #brand {{ color: {TEXT}; font-size: 21px; font-weight: 700; }}
        #product, #sidebarFooter {{ color: #86868b; font-size: 13px; }}
        #nav, #stepList {{ border: 0; background: transparent; outline: 0; }}
        #nav::item {{ color: #515154; padding: 13px 12px; border-radius: 8px; margin-bottom: 4px; }}
        #nav::item:hover {{ background: #f0f0f3; color: {TEXT}; }}
        #nav::item:selected {{ background: #e8e8ed; color: {TEXT}; font-weight: 600; }}
        #stepList::item {{ background: transparent; padding: 13px 8px; color: {MUTED}; border-radius: 8px; }}
        #stepList::item:selected {{ background: #eaf3fc; color: {BLUE}; }}
        #topbar {{ background: rgba(255, 255, 255, 245); border-bottom: 1px solid #e5e5ea; }}
        #pageTitle {{ font-size: 26px; font-weight: 700; }}
        #sectionTitle {{ font-size: 20px; font-weight: 700; }}
        #subtitle {{ color: {MUTED}; }}
        #card {{ background: {CARD}; border: 1px solid #e5e5ea; border-radius: 8px; }}
        #cardTitle {{ font-weight: 600; font-size: 16px; }}
        #statusValue, #summary, #result {{ color: {MUTED}; }}
        QLabel[status='ok'], #connection[status='ok'] {{ color: {GREEN}; }}
        QLabel[status='bad'], #connection[status='bad'] {{ color: {RED}; }}
        #notice {{ background: #eef6ff; color: #0066cc; border-radius: 8px; padding: 11px 14px; }}
        #notice[notice='warning'] {{ background: #fff2f2; color: #b42318; }}
        #notice[notice='success'] {{ background: #eef8f0; color: #1f7a36; }}
        #preview {{ background: #f2f2f7; border-radius: 8px; min-height: 270px; }}
        #handIcon {{ font-size: 92px; }}
        #previewText {{ color: {MUTED}; }}
        QPushButton {{ background: #e8e8ed; border: 0; border-radius: 8px; min-height: 24px; padding: 8px 16px; }}
        QPushButton:hover {{ background: #dedee3; }}
        QPushButton:pressed {{ background: #d1d1d6; }}
        QPushButton:disabled {{ color: #aeaeb2; background: #f0f0f3; }}
        #primary {{ background: {BLUE}; color: white; font-weight: 600; }}
        #primary:hover {{ background: #0077ed; }}
        #primary:pressed {{ background: #0068d1; }}
        #primary:disabled {{ background: #d9e8f6; color: #8aaccb; }}
        QProgressBar {{ border: 0; background: #e8e8ed; border-radius: 5px; height: 9px; text-align: center; }}
        QProgressBar::chunk {{ background: {BLUE}; border-radius: 5px; }}
        #handSelector {{ background: #e8e8ed; border: 1px solid #dedee3; border-radius: 8px; }}
        #handChoice {{ background: transparent; color: {MUTED}; border: 0; border-radius: 6px; min-width: 48px; padding: 6px 12px; }}
        #handChoice:hover {{ background: #f5f5f7; }}
        #handChoice:checked {{ background: white; color: {BLUE}; border: 1px solid #d2d2d7; font-weight: 600; }}
        #handChoice:checked:disabled {{ background: #f5f5f7; color: #5f8fb8; border: 1px solid #dedee3; }}
        #handStatus {{ color: {MUTED}; font-size: 12px; min-width: 60px; }}
        """
    )


if __name__ == "__main__":
    raise SystemExit(main())
