#!/usr/bin/env python3
import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent))

from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QBoxLayout, QListWidget

from manus_l20_calibration_ui import ASSET_DIR, CalibrationPage, MainWindow, ManusGloveCalibrationPage, SystemPage


def immediate(operation, succeeded, failed) -> None:
    try:
        succeeded(operation())
    except Exception as exc:
        failed(str(exc))


def ready_page(**kwargs) -> CalibrationPage:
    page = CalibrationPage(**kwargs)
    page.update_glove_topics({"/manus_glove_0"})
    return page


class FakeFullSession:
    def __init__(self) -> None:
        self.started = False
        self.closed = False
        self.poses: list[str] = []

    def start(self) -> None:
        self.started = True

    def capture_pose(self, label: str) -> None:
        self.poses.append(label)

    def save(self):
        return [f"/tmp/full-{index}.yaml" for index in range(4)]

    def close(self) -> None:
        self.closed = True


class FakeContactSession:
    def __init__(self) -> None:
        self.closed = False
        self.poses: list[str] = []

    def capture_pose(self, label: str) -> None:
        self.poses.append(label)

    def save(self):
        return "/tmp/contact.yaml"

    def close(self) -> None:
        self.closed = True


class CalibrationPageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_integrated_eight_step_flow_uses_assets_and_saves(self) -> None:
        full = FakeFullSession()
        contact = FakeContactSession()
        page = ready_page(
            full_factory=lambda **_kwargs: full,
            contact_factory=lambda **_kwargs: contact,
            task_runner=immediate,
        )
        self.assertEqual(len(page.STEPS), 8)
        self.assertFalse(page.gesture_image.pixmap().isNull())
        self.assertEqual(page.step, 0)
        self.assertFalse(page.next_button.isEnabled())
        page.capture()
        self.assertTrue(full.started)
        self.assertTrue(page.next_button.isEnabled())

        for expected_pose in page.FULL_POSES.values():
            page.next_step()
            page.capture()
            self.assertEqual(full.poses[-1], expected_pose)
            self.assertTrue(page.next_button.isEnabled())
        self.assertTrue(full.closed)
        self.assertEqual(len(page.saved_paths), 4)

        page.next_step()
        self.assertEqual(page.step, 6)
        for expected_pose in page.CONTACT_POSES:
            page.capture()
            self.assertEqual(contact.poses[-1], expected_pose)
        self.assertTrue(contact.closed)
        self.assertTrue(page.next_button.isEnabled())
        page.next_step()
        self.assertEqual(page.step, 7)
        self.assertEqual(len(page.saved_paths), 5)

    def test_start_failure_recovers_button(self) -> None:
        def fail_factory(**_kwargs):
            raise RuntimeError("MANUS topic unavailable")

        page = ready_page(full_factory=fail_factory, task_runner=immediate)
        page.capture()
        self.assertTrue(page.action.isEnabled())
        self.assertIn("MANUS topic unavailable", page.result.text())

    def test_step_list_cannot_skip_active_step(self) -> None:
        page = ready_page(task_runner=immediate)
        page.resize(900, 650)
        page.show()
        self.app.processEvents()
        page.step = 1
        page.update_step()

        target = page.step_list.visualItemRect(page.step_list.item(4)).center()
        QTest.mouseClick(page.step_list.viewport(), Qt.LeftButton, pos=target)
        self.app.processEvents()

        self.assertEqual(page.step, 1)
        self.assertEqual(page.step_list.currentRow(), 1)

    def test_previous_step_rolls_back_and_can_return_to_device_selection(self) -> None:
        full = FakeFullSession()
        page = ready_page(full_factory=lambda **_kwargs: full, task_runner=immediate)
        page.capture()
        page.next_step()
        self.assertTrue(page.back_button.isEnabled())
        page.capture()
        page.next_step()

        page.previous_step()
        self.assertEqual(page.step, 1)
        self.assertEqual(page.progress, 0)
        self.assertEqual(page.full_session, full)

        page.previous_step()
        self.assertEqual(page.step, 0)
        self.assertFalse(page.calibration_started)
        self.assertIsNone(page.full_session)
        self.assertTrue(full.closed)
        self.assertEqual(page.selected_hand, "right")
        self.assertFalse(page.hand_buttons["right"].isEnabled())

    def test_restart_closes_sessions_and_resets_workflow(self) -> None:
        full = FakeFullSession()
        contact = FakeContactSession()
        page = ready_page(task_runner=immediate)
        page.update_glove_topics({"/manus_glove_1"})
        page.full_session = full
        page.contact_session = contact
        page.calibration_started = True
        page.step = 6
        page.progress = 60
        page.contact_index = 3
        page.saved_paths = ["full.yaml", "contact.yaml"]
        page.update_step()

        page.restart_calibration()

        self.assertTrue(full.closed)
        self.assertTrue(contact.closed)
        self.assertEqual(page.step, 0)
        self.assertEqual(page.progress, 0)
        self.assertEqual(page.contact_index, 0)
        self.assertEqual(page.saved_paths, [])
        self.assertFalse(page.calibration_started)
        self.assertFalse(page.hand_buttons["left"].isEnabled())
        self.assertEqual(page.selected_hand, "left")
        self.assertFalse(page.restart_button.isEnabled())

    def test_contact_stage_rollback_discards_partial_contact_flow(self) -> None:
        full = FakeFullSession()
        contact = FakeContactSession()
        page = ready_page(task_runner=immediate)
        page.full_session = full
        page.contact_session = contact
        page.calibration_started = True
        page.step = 6
        page.progress = 40
        page.contact_index = 2
        page.saved_paths = full.save() + ["/tmp/contact.yaml"]

        page.previous_step()

        self.assertTrue(contact.closed)
        self.assertEqual(page.step, 5)
        self.assertEqual(page.contact_index, 0)
        self.assertIsNone(page.contact_session)
        self.assertEqual(len(page.saved_paths), 4)
        page.capture()
        self.assertEqual(full.poses[-1], "thumb_pinky_root_touch")
        self.assertEqual(len(page.saved_paths), 4)

    def test_busy_state_locks_navigation_and_restart(self) -> None:
        page = CalibrationPage(task_runner=lambda *_args: None)
        page.update_glove_topics({"/manus_glove_0", "/manus_glove_1"})
        QTest.mouseClick(page.hand_buttons["right"], Qt.LeftButton)
        page.capture()

        self.assertTrue(page.busy)
        self.assertFalse(page.step_list.hasFocus())
        self.assertFalse(page.next_button.isEnabled())
        self.assertFalse(page.back_button.isEnabled())
        self.assertFalse(page.restart_button.isEnabled())
        self.assertFalse(page.hand_buttons["left"].isEnabled())
        self.assertFalse(page.hand_buttons["right"].isEnabled())

    def test_compact_window_keeps_controls_beside_responsive_image(self) -> None:
        page = ready_page(task_runner=immediate)
        page.resize(682, 597)
        page.show()
        self.app.processEvents()

        self.assertEqual(page.current_layout.direction(), QBoxLayout.LeftToRight)
        pixmap = page.gesture_image.pixmap()
        side = min(page.gesture_image.width(), page.gesture_image.height())
        self.assertLessEqual(pixmap.width(), side)
        self.assertLessEqual(pixmap.height(), side)
        self.assertEqual(max(pixmap.width(), pixmap.height()), side)
        self.assertGreaterEqual(page.restart_button.height(), 30)

    def test_all_l20_steps_use_existing_gesture_assets(self) -> None:
        page = CalibrationPage(task_runner=immediate)
        filenames = set(page.STEP_IMAGES.values()) | set(page.CONTACT_IMAGES)

        self.assertEqual(len(filenames), 6)
        self.assertTrue(all((ASSET_DIR / filename).is_file() for filename in filenames))

        page.step = 5
        page.update_step()
        self.assertFalse(page.gesture_image.pixmap().isNull())
        page.step = 6
        for contact_index in range(len(page.CONTACT_IMAGES)):
            page.contact_index = contact_index
            page.update_step()
            self.assertIs(page.preview_stack.currentWidget(), page.gesture_image)
            self.assertFalse(page.gesture_image.pixmap().isNull())

    def test_glove_topics_auto_select_single_hand_and_require_choice_for_both(self) -> None:
        page = CalibrationPage(task_runner=immediate)
        self.assertIsNone(page.selected_hand)
        self.assertFalse(page.action.isEnabled())
        self.assertEqual(page.hand_status.text(), "未检测到")

        page.update_glove_topics({"/manus_glove_1"})
        self.assertEqual(page.selected_hand, "left")
        self.assertEqual(page._session_options()["glove_topic"], "/manus_glove_1")
        self.assertTrue(page.hand_buttons["left"].isChecked())
        self.assertFalse(page.hand_buttons["left"].isEnabled())
        self.assertTrue(page.action.isEnabled())

        page.update_glove_topics({"/manus_glove_0", "/manus_glove_1"})
        self.assertTrue(page.hand_buttons["left"].isEnabled())
        self.assertTrue(page.hand_buttons["right"].isEnabled())
        QTest.mouseClick(page.hand_buttons["right"], Qt.LeftButton)
        self.assertEqual(page.selected_hand, "right")
        self.assertEqual(page._session_options()["glove_topic"], "/manus_glove_0")

    def test_two_gloves_on_first_detection_require_explicit_selection(self) -> None:
        page = CalibrationPage(task_runner=immediate)
        page.update_glove_topics({"/manus_glove_0", "/manus_glove_1"})

        self.assertIsNone(page.selected_hand)
        self.assertEqual(page.hand_status.text(), "请选择")
        self.assertFalse(page.action.isEnabled())
        QTest.mouseClick(page.hand_buttons["left"], Qt.LeftButton)
        self.assertEqual(page.selected_hand, "left")
        self.assertTrue(page.action.isEnabled())

    def test_system_status_returns_topics_used_for_glove_detection(self) -> None:
        page = SystemPage()
        received = []
        page.topics_ready.connect(received.append)
        page._probe_active = True

        page._complete_probe({"/manus_glove_0", "/manus_glove_1"})

        self.assertEqual(received, [{"/manus_glove_0", "/manus_glove_1"}])
        self.assertEqual(page.rows["ros"][0].property("status"), "ok")
        self.assertEqual(page.rows["manus"][0].property("status"), "ok")

    def test_hotplug_changes_show_persistent_notice_and_lock_capture(self) -> None:
        page = CalibrationPage(task_runner=immediate)
        page.update_glove_topics(set())
        self.assertIn("未检测到", page.banner.text())
        self.assertEqual(page.banner.property("notice"), "warning")

        page.update_glove_topics({"/manus_glove_0"})
        self.assertIn("已自动识别右手", page.banner.text())
        self.assertEqual(page.banner.property("notice"), "success")
        self.assertTrue(page.action.isEnabled())

        page.update_glove_topics(set())
        self.assertIn("右手手套连接已断开", page.banner.text())
        self.assertEqual(page.banner.property("notice"), "warning")
        self.assertFalse(page.action.isEnabled())

    def test_manus_glove_sdk_protocol_uses_dynamic_steps_and_finishes(self) -> None:
        page = ManusGloveCalibrationPage(auto_start_bridge=False)
        page.bridge_ready = True
        page.handle_status("GLOVES\t101\t202\t7\t4")
        self.assertIsNone(page.selected_hand)
        QTest.mouseClick(page.hand_buttons["left"], Qt.LeftButton)
        self.assertEqual(page.selected_hand, "left")

        page.handle_status("STARTED\tleft\t101\t3\t7")
        page.handle_status("STEP_READY\t0\t3\tOpen hand\tKeep still\t5.0")
        self.assertEqual(page.step_list.count(), 3)
        self.assertEqual(page.step_title.text(), "1. 张开手掌")
        self.assertTrue(page.action.isEnabled())

        page.handle_status("CAPTURING\t0\t3")
        self.assertFalse(page.action.isEnabled())
        page.handle_status("STEP_DONE\t0\t3")
        self.assertTrue(page.next_button.isEnabled())
        page.handle_status("FINISHED\t/tmp/Calibration_left.mcal")
        QTest.qWait(300)
        self.assertIn("Calibration_left.mcal", page.result.text())
        self.assertEqual(page.progress_bar.value(), 100)

    def test_manus_glove_preview_parses_and_renders_3d_skeletons(self) -> None:
        page = ManusGloveCalibrationPage(auto_start_bridge=False)
        left = "0,-1,0,0,0;1,0,0.02,0.01,0.03;2,1,0.04,0.02,0.06"
        right = "0,-1,0,0,0;1,0,-0.02,0.01,0.03;2,1,-0.04,0.02,0.06"
        page.handle_status(f"SKELETON\tleft\t{left}")
        page.handle_status(f"SKELETON\tright\t{right}")
        page.resize(900, 620)
        page.show()
        self.app.processEvents()

        self.assertEqual(page.skeleton.visible_hands(), ["left", "right"])
        self.assertFalse(page.skeleton.grab().isNull())
        self.assertIs(page.preview_stack.currentWidget(), page.skeleton)

        page.handle_status("STARTED\tleft\t101\t4\t7")
        page.handle_status("STEP_READY\t0\t4\tOpen hand\tKeep still\t5.0")
        self.assertIs(page.preview_stack.currentWidget(), page.movie)
        page.handle_status("STOPPED")
        self.assertIs(page.preview_stack.currentWidget(), page.skeleton)

    def test_skeleton_frames_do_not_resync_unrelated_controls(self) -> None:
        page = ManusGloveCalibrationPage(auto_start_bridge=False)
        sync_calls = []
        page._sync_controls = lambda: sync_calls.append(True)

        page.handle_status("SKELETON\tleft\t0,-1,0,0,0;1,0,0.02,0.01,0.03")

        self.assertEqual(sync_calls, [])
        self.assertEqual(page.skeleton.visible_hands(), ["left"])

    def test_main_window_contains_only_two_calibration_tabs(self) -> None:
        window = MainWindow(auto_start_bridge=False)
        self.assertEqual(window.pages.count(), 2)
        self.assertIsInstance(window.pages.widget(0), ManusGloveCalibrationPage)
        self.assertIsInstance(window.pages.widget(1), CalibrationPage)
        self.assertEqual(window.size().width(), 1360)
        self.assertEqual(window.size().height(), 840)
        self.assertEqual(window.windowTitle(), "灵巧手标定")
        window.close()

    def test_rapid_tab_switch_does_not_apply_page_graphics_effect(self) -> None:
        window = MainWindow(auto_start_bridge=False)
        window.show()
        nav = window.findChild(QListWidget, "nav")
        self.assertIsNotNone(nav)
        for index in (1, 0, 1, 0, 1):
            nav.setCurrentRow(index)
            QApplication.processEvents()
        self.assertEqual(window.pages.currentIndex(), 1)
        self.assertIsNone(window.pages.widget(0).graphicsEffect())
        self.assertIsNone(window.pages.widget(1).graphicsEffect())
        QTest.qWait(200)
        self.assertEqual(window.pages.currentWidget().pos(), QPoint(0, 0))
        window.close()


if __name__ == "__main__":
    unittest.main()
