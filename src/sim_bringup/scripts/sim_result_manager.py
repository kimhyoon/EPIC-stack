#!/usr/bin/env python3

"""Run the unmodified EPIC evaluators and organize one simulation result.

Metric collection uses EPIC's existing scripts. This node only owns their
lifecycle, defers plotting until FINISH, and copies generated artifacts into a
per-test result directory.
"""

import hashlib
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path

import rospy
import rospkg
from visualization_msgs.msg import Marker


class SimResultManager:
    def __init__(self):
        rospy.init_node("sim_result_manager")

        self.result_root = Path(os.path.abspath(os.path.expanduser(
            rospy.get_param("~result_root", "/workspace/sim_validation_ws/result"))))
        self.tmp_root = Path(os.path.abspath(os.path.expanduser(
            rospy.get_param("~tmp_root", "/dev/shm/epic_sim_results"))))
        self.test_prefix = rospy.get_param("~test_prefix", "test")
        self.experiment_name = rospy.get_param("~experiment_name", "epic_planning")
        self.eval_flush_timeout = float(rospy.get_param("~eval_flush_timeout", 15.0))

        self.odom_topic = rospy.get_param("~odom_topic")
        self.position_cmd_topic = rospy.get_param(
            "~position_cmd_topic", "/planning/pos_cmd")
        self.pcd_path = rospy.get_param("~pcd_path")
        self.cell_size = float(rospy.get_param("~cell_size", 0.4))
        self.map_min = [
            float(rospy.get_param("~map_min_x", -10.0)),
            float(rospy.get_param("~map_min_y", -6.0)),
            float(rospy.get_param("~map_min_z", -0.2)),
        ]
        self.map_max = [
            float(rospy.get_param("~map_max_x", 182.0)),
            float(rospy.get_param("~map_max_y", 150.2)),
            float(rospy.get_param("~map_max_z", 3.8)),
        ]

        scripts_dir = Path(rospkg.RosPack().get_path("epic_planner")) / "scripts"
        self.evaluate_script = scripts_dir / "evaluate_exploration.py"
        self.metrics_script = scripts_dir / "measure_planning_metrics.py"
        self.plot_evaluation_script = scripts_dir / "plot_experiment_results.py"
        self.plot_planning_script = scripts_dir / "plot_planning_time.py"
        self._verify_source_scripts()

        self.result_root.mkdir(parents=True, exist_ok=True)
        try:
            self.tmp_root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            fallback = Path("/tmp/epic_sim_results")
            fallback.mkdir(parents=True, exist_ok=True)
            rospy.logwarn("[SimResult] cannot use %s (%s); using %s",
                          self.tmp_root, exc, fallback)
            self.tmp_root = fallback

        self.test_id = self._new_test_id()
        self.final_dir = self.result_root / self.test_id
        self.work_dir = Path(tempfile.mkdtemp(
            prefix=self.test_id + "_", dir=str(self.tmp_root)))
        self.eval_dir = self.work_dir / "evaluation"
        self.eval_iter_dir = self.eval_dir / "iter_1"
        self.planning_dir = self.work_dir / "planning"
        self.eval_iter_dir.mkdir(parents=True)
        self.planning_dir.mkdir(parents=True)

        self.children = []
        self.prev_state = ""
        self.wait_trigger_seen = False
        self.test_active = False
        self.finish_seen = False
        self.finalizing = False
        self.finalized = False
        self.lock = threading.Lock()

        # The original EPIC evaluator starts on WAIT_TRIGGER -> PLAN_TRAJ_EXP.
        # MARSIM inserts TAKEOFF_HOVER between those states, so publish a
        # compatibility state stream for the evaluator without changing its
        # metric or plotting implementation.
        self.eval_state_pub = rospy.Publisher(
            "/sim_result_manager/eval_state", Marker, queue_size=10, latch=True)

        self._start_collectors()
        self.state_sub = rospy.Subscriber(
            "/planning/state", Marker, self._state_callback, queue_size=10)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[SimResult] armed for one test: %s", self.test_id)
        rospy.loginfo("[SimResult] runtime files use %s", self.work_dir)
        rospy.loginfo("[SimResult] final output will be %s", self.final_dir)

    def _verify_source_scripts(self):
        scripts = [
            self.evaluate_script,
            self.metrics_script,
            self.plot_evaluation_script,
            self.plot_planning_script,
        ]
        missing = [str(path) for path in scripts if not path.is_file()]
        if missing:
            raise RuntimeError("missing EPIC evaluation scripts: " + ", ".join(missing))

    def _new_test_id(self):
        base = "%s_%s" % (
            self.test_prefix, datetime.now().strftime("%Y%m%d_%H%M%S"))
        candidate = base
        index = 1
        while (self.result_root / candidate).exists():
            candidate = "%s_%02d" % (base, index)
            index += 1
        return candidate

    @staticmethod
    def _ros_private(name, value):
        if isinstance(value, bool):
            value = "true" if value else "false"
        return "_%s:=%s" % (name, value)

    def _start_collectors(self):
        eval_cmd = [
            sys.executable,
            str(self.evaluate_script),
            "__name:=exploration_evaluator",
            "/planning/state:=/sim_result_manager/eval_state",
            self._ros_private("odom_topic", self.odom_topic),
            self._ros_private("position_cmd_topic", self.position_cmd_topic),
            self._ros_private("pcd_path", self.pcd_path),
            self._ros_private("cell_size", self.cell_size),
            self._ros_private("map_min_x", self.map_min[0]),
            self._ros_private("map_min_y", self.map_min[1]),
            self._ros_private("map_min_z", self.map_min[2]),
            self._ros_private("map_max_x", self.map_max[0]),
            self._ros_private("map_max_y", self.map_max[1]),
            self._ros_private("map_max_z", self.map_max[2]),
            self._ros_private("output_dir", self.eval_iter_dir),
            self._ros_private("auto_shutdown", False),
        ]
        metrics_cmd = [
            sys.executable,
            str(self.metrics_script),
            "__name:=planning_metrics_measurer",
            self._ros_private("output_dir", self.planning_dir),
            self._ros_private("experiment_name", self.experiment_name),
        ]

        for name, command in (("evaluation", eval_cmd), ("planning", metrics_cmd)):
            process = subprocess.Popen(command, preexec_fn=os.setsid)
            self.children.append((name, process))
            rospy.loginfo("[SimResult] started original EPIC %s collector (pid=%d)",
                          name, process.pid)

    def _state_callback(self, msg):
        state = (msg.text or "").strip()
        if not state:
            return

        if state == "WAIT_TRIGGER":
            self.wait_trigger_seen = True

        # Keep TAKEOFF_HOVER out of the evaluator's legacy state stream. The
        # latched publisher also preserves WAIT_TRIGGER while the evaluator
        # finishes loading the PCD and subscribes.
        if not (state == "TAKEOFF_HOVER" and not self.test_active):
            self.eval_state_pub.publish(msg)

        if state == self.prev_state:
            return

        if (not self.test_active and self.wait_trigger_seen and
                state == "PLAN_TRAJ_EXP"):
            self.test_active = True
            rospy.loginfo("[SimResult] test measurement started")

        if self.test_active and state == "FINISH" and not self.finish_seen:
            self.finish_seen = True
            rospy.loginfo("[SimResult] FINISH received; sealing metrics before plotting")
            threading.Thread(
                target=self._finalize,
                args=("COMPLETE",),
                name="sim_result_finalize",
                daemon=False,
            ).start()

        self.prev_state = state

    def _wait_for_evaluation(self):
        summary = self.eval_iter_dir / "summary.csv"
        deadline = time.monotonic() + self.eval_flush_timeout
        while time.monotonic() < deadline and not summary.is_file():
            time.sleep(0.1)
        return summary.is_file()

    @staticmethod
    def _stop_process(name, process):
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except Exception as exc:
            rospy.logwarn("[SimResult] %s collector did not stop cleanly: %s", name, exc)
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except Exception:
                pass

    def _stop_collectors(self):
        for name, process in self.children:
            self._stop_process(name, process)
        self.children = []

    def _run_plot(self, command, log_file):
        env = os.environ.copy()
        env["MPLBACKEND"] = "Agg"
        with log_file.open("a", encoding="utf-8") as stream:
            stream.write("\n$ %s\n" % " ".join(command))
            stream.flush()
            result = subprocess.run(
                command,
                stdout=stream,
                stderr=subprocess.STDOUT,
                env=env,
                check=False,
            )
            stream.write("exit_code=%d\n" % result.returncode)
        return result.returncode

    @staticmethod
    def _copy_artifacts(source_root, pattern, destination):
        destination.mkdir(parents=True, exist_ok=True)
        copied = []
        for source in sorted(source_root.rglob(pattern)):
            target = destination / source.name
            if target.exists():
                target = destination / (source.parent.name + "_" + source.name)
            shutil.copy2(str(source), str(target))
            copied.append(target.name)
        return copied

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _write_source_manifest(self, destination):
        scripts = [
            self.evaluate_script,
            self.metrics_script,
            self.plot_evaluation_script,
            self.plot_planning_script,
        ]
        with destination.open("w", encoding="ascii") as stream:
            for path in scripts:
                stream.write("%s  %s\n" % (self._sha256(path), path))

    def _finalize(self, status):
        with self.lock:
            if self.finalizing or self.finalized:
                return
            self.finalizing = True

        evaluation_ready = False
        try:
            if status == "COMPLETE":
                evaluation_ready = self._wait_for_evaluation()

            self._stop_collectors()
            if not evaluation_ready:
                evaluation_ready = (self.eval_iter_dir / "summary.csv").is_file()

            self.final_dir.mkdir(parents=True, exist_ok=True)
            log_file = self.final_dir / "postprocess.log"

            evaluation_exit = self._run_plot(
                [sys.executable, str(self.plot_evaluation_script), str(self.eval_dir)],
                log_file,
            ) if evaluation_ready else -1

            planning_exit = self._run_plot(
                [
                    sys.executable,
                    str(self.plot_planning_script),
                    "--data_dir", str(self.planning_dir),
                    "--experiment", self.experiment_name,
                    "--no_show",
                ],
                log_file,
            )

            csv_files = self._copy_artifacts(
                self.work_dir, "*.csv", self.final_dir / "csv_file")
            png_files = self._copy_artifacts(
                self.work_dir, "*.png", self.final_dir / "png")
            self._write_source_manifest(self.final_dir / "source_scripts.sha256")

            with (self.final_dir / "status.txt").open("w", encoding="ascii") as stream:
                stream.write("status=%s\n" % status)
                stream.write("test_id=%s\n" % self.test_id)
                stream.write("evaluation_summary=%s\n" % (
                    "present" if evaluation_ready else "missing"))
                stream.write("evaluation_plot_exit=%d\n" % evaluation_exit)
                stream.write("planning_plot_exit=%d\n" % planning_exit)
                stream.write("csv_count=%d\n" % len(csv_files))
                stream.write("png_count=%d\n" % len(png_files))

            rospy.loginfo("[SimResult] result finalized: %s", self.final_dir)
            rospy.loginfo("[SimResult] generated %d CSV and %d PNG files",
                          len(csv_files), len(png_files))
        except Exception as exc:
            rospy.logerr("[SimResult] finalization failed: %s", exc)
        finally:
            shutil.rmtree(str(self.work_dir), ignore_errors=True)
            with self.lock:
                self.finalized = True
                self.finalizing = False

    def _on_shutdown(self):
        if self.finalized:
            return
        if self.test_active:
            self._finalize("PARTIAL")
        else:
            self._stop_collectors()
            shutil.rmtree(str(self.work_dir), ignore_errors=True)


def main():
    try:
        SimResultManager()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logfatal("[SimResult] startup failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
