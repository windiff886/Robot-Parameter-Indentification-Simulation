import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.connected = False
        self.enable_attempts = 0

    def ConnectPort(self):
        self.connected = True
        self.calls.append(("ConnectPort",))

    def DisconnectPort(self):
        self.connected = False
        self.calls.append(("DisconnectPort",))

    def EnablePiper(self):
        self.enable_attempts += 1
        self.calls.append(("EnablePiper",))
        return self.enable_attempts >= 2

    def DisablePiper(self):
        self.calls.append(("DisablePiper",))
        return True

    def EmergencyStop(self, code):
        self.calls.append(("EmergencyStop", code))

    def MotionCtrl_2(self, ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode):
        self.calls.append(
            ("MotionCtrl_2", ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode)
        )

    def JointCtrl(self, *joints):
        self.calls.append(("JointCtrl",) + joints)

    def JointMitCtrl(self, motor_num, pos_ref, vel_ref, kp, kd, t_ref):
        self.calls.append(("JointMitCtrl", motor_num, pos_ref, vel_ref, kp, kd, t_ref))

    def GetArmJointMsgs(self):
        return SimpleNamespace(
            joint_state=SimpleNamespace(
                joint_1=1000,
                joint_2=2000,
                joint_3=-3000,
                joint_4=0,
                joint_5=500,
                joint_6=-500,
            )
        )

    def GetArmHighSpdInfoMsgs(self):
        return SimpleNamespace(
            motor_1=SimpleNamespace(motor_speed=100, effort=1000),
            motor_2=SimpleNamespace(motor_speed=200, effort=2000),
            motor_3=SimpleNamespace(motor_speed=-300, effort=-3000),
            motor_4=SimpleNamespace(motor_speed=0, effort=0),
            motor_5=SimpleNamespace(motor_speed=50, effort=500),
            motor_6=SimpleNamespace(motor_speed=-50, effort=-500),
        )

    def GetArmStatus(self):
        return SimpleNamespace(
            arm_status=SimpleNamespace(
                arm_status=0,
                ctrl_mode=1,
                mode_feed=1,
                err_status=SimpleNamespace(
                    joint_1_angle_limit=False,
                    joint_2_angle_limit=False,
                    joint_3_angle_limit=False,
                    joint_4_angle_limit=False,
                    joint_5_angle_limit=False,
                    joint_6_angle_limit=False,
                    communication_status_joint_1=False,
                    communication_status_joint_2=False,
                    communication_status_joint_3=False,
                    communication_status_joint_4=False,
                    communication_status_joint_5=False,
                    communication_status_joint_6=False,
                ),
            )
        )


class ConfigLoaderTest(unittest.TestCase):
    def test_loads_real_experiment_yaml(self):
        from piper_real.config import load_real_experiment_config

        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "real.yaml"
            config_path.write_text(
                "\n".join(
                    [
                        'can_port: "can_left"',
                        "control_mode: mit_identification",
                        "control_rate_hz: 200.0",
                        "record_rate_hz: 100.0",
                        'output_csv: "data/real.csv"',
                        "duration: 12.5",
                        "speed_rate: 40",
                        "home_position: [0.0, 0.3, -0.2, 0.0, 0.1, -0.1]",
                        "position_amplitude: [0.1, 0.1, 0.05, 0.05, 0.02, 0.02]",
                        "position_frequency: [0.2, 0.2, 0.3, 0.3, 0.4, 0.4]",
                        "mit_kp: [12, 12, 10, 8, 6, 6]",
                        "mit_kd: [0.8, 0.8, 0.7, 0.5, 0.4, 0.4]",
                        "max_samples: 50",
                        "joint_soft_limits: [[-1, 1], [-2, 2], [-3, 3], [-4, 4], [-5, 5], [-6, 6]]",
                    ]
                ),
                encoding="utf-8",
            )

            config = load_real_experiment_config(config_path, repo_root=REPO_ROOT)

        self.assertEqual(config.can_port, "can_left")
        self.assertEqual(config.control_mode, "mit_identification")
        self.assertEqual(config.speed_rate, 40)
        self.assertEqual(config.output_csv, REPO_ROOT / "data" / "real.csv")
        self.assertEqual(config.max_samples, 50)
        self.assertEqual(config.home_position[1], 0.3)
        self.assertEqual(config.joint_soft_limits[2], (-3.0, 3.0))


class AdapterTest(unittest.TestCase):
    def test_reads_joint_state_and_converts_units(self):
        from piper_real.sdk_adapter import PiperSdkAdapter

        adapter = PiperSdkAdapter(sdk_factory=lambda can_port: FakeSdk(), can_port="can0")
        state = adapter.read_state()

        self.assertAlmostEqual(state.position[0], math.radians(1.0), places=9)
        self.assertAlmostEqual(state.position[2], math.radians(-3.0), places=9)
        self.assertAlmostEqual(state.velocity[0], 0.1, places=9)
        self.assertAlmostEqual(state.velocity[2], -0.3, places=9)
        self.assertAlmostEqual(state.effort[1], 2.0, places=9)

    def test_sends_position_command_in_sdk_units(self):
        from piper_real.sdk_adapter import PiperSdkAdapter

        fake_sdk = FakeSdk()
        adapter = PiperSdkAdapter(sdk_factory=lambda can_port: fake_sdk, can_port="can0")
        adapter.send_joint_position([0.1, -0.2, 0.0, 0.3, -0.4, 0.5], speed_rate=35)

        self.assertEqual(fake_sdk.calls[0], ("MotionCtrl_2", 0x01, 0x01, 35, 0x00))
        expected_joint_1 = round(math.degrees(0.1) * 1000.0)
        expected_joint_6 = round(math.degrees(0.5) * 1000.0)
        self.assertEqual(
            fake_sdk.calls[1],
            ("JointCtrl", expected_joint_1, round(math.degrees(-0.2) * 1000.0), 0, round(math.degrees(0.3) * 1000.0), round(math.degrees(-0.4) * 1000.0), expected_joint_6),
        )


class RecorderTest(unittest.TestCase):
    def test_writes_identification_csv_with_estimated_qdd(self):
        from piper_real.recorder import CsvRecorder, RealSample

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "record.csv"
            recorder = CsvRecorder(csv_path, dof=2)
            recorder.record(
                RealSample(
                    timestamp=0.0,
                    position=[0.0, 1.0],
                    velocity=[0.0, 0.5],
                    effort=[1.0, 2.0],
                    commanded_effort=[1.1, 2.1],
                )
            )
            recorder.record(
                RealSample(
                    timestamp=0.1,
                    position=[0.1, 1.2],
                    velocity=[1.0, 1.5],
                    effort=[1.2, 2.2],
                    commanded_effort=[1.3, 2.3],
                )
            )
            recorder.close()

            with csv_path.open(encoding="utf-8") as handle:
                rows = list(csv.reader(handle))

        self.assertEqual(
            rows[0],
            ["time", "q0", "q1", "qd0", "qd1", "qdd0", "qdd1", "tau0", "tau1", "tau_cmd0", "tau_cmd1"],
        )
        self.assertEqual(rows[1][0], "0.000000")
        self.assertEqual(rows[1][5], "0.000000")
        self.assertEqual(rows[2][5], "10.000000")
        self.assertEqual(rows[2][8], "2.200000")


class TrajectoryTest(unittest.TestCase):
    def test_fourier_excitation_respects_initial_conditions(self):
        from piper_real.trajectory import FourierExcitationTrajectory

        trajectory = FourierExcitationTrajectory(
            q0=[0.1, -0.2, 0.3, 0.0, 0.05, -0.05],
            period=10.0,
            a_coefficients=[
                [0.2, -0.1, -0.1],
                [0.3, -0.2, -0.1],
                [0.1, 0.0, -0.1],
                [0.05, -0.04, -0.01],
                [0.02, -0.01, -0.01],
                [0.02, -0.01, -0.01],
            ],
            b_coefficients=[
                [0.0, 0.1, -0.2],
                [0.0, 0.2, -0.4],
                [0.0, -0.1, 0.2],
                [0.0, 0.02, -0.04],
                [0.0, 0.01, -0.02],
                [0.0, 0.01, -0.02],
            ],
        )

        point = trajectory.evaluate(0.0)

        self.assertAlmostEqual(point.position[0], 0.1, places=9)
        self.assertAlmostEqual(point.position[1], -0.2, places=9)
        self.assertAlmostEqual(point.velocity[0], 0.0, places=9)
        self.assertAlmostEqual(point.velocity[3], 0.0, places=9)


class RunnerTest(unittest.TestCase):
    def test_runner_executes_position_validation_and_records_samples(self):
        from piper_real.config import RealExperimentConfig
        from piper_real.recorder import CsvRecorder
        from piper_real.runner import RealExperimentRunner

        fake_sdk = FakeSdk()
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "run.csv"
            config = RealExperimentConfig(
                can_port="can0",
                control_mode="position_validation",
                control_rate_hz=100.0,
                record_rate_hz=100.0,
                output_csv=csv_path,
                duration=0.03,
                speed_rate=25,
                home_position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                position_amplitude=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                position_frequency=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
                mit_kp=[10.0] * 6,
                mit_kd=[0.8] * 6,
                max_samples=2,
            )
            runner = RealExperimentRunner(
                config=config,
                adapter_factory=lambda cfg: __import__("piper_real.sdk_adapter", fromlist=["PiperSdkAdapter"]).PiperSdkAdapter(
                    sdk_factory=lambda can_port: fake_sdk,
                    can_port=cfg.can_port,
                ),
                recorder_factory=lambda path, dof: CsvRecorder(path, dof),
                sleep_fn=lambda _: None,
            )

            runner.run()

            with csv_path.open(encoding="utf-8") as handle:
                rows = list(csv.reader(handle))

        self.assertGreaterEqual(len(rows), 3)
        self.assertIn(("ConnectPort",), fake_sdk.calls)
        self.assertIn(("EnablePiper",), fake_sdk.calls)
        self.assertTrue(any(call[0] == "JointCtrl" for call in fake_sdk.calls))
        self.assertEqual(fake_sdk.calls[-1], ("DisconnectPort",))

    def test_runner_executes_excitation_trajectory_with_mit_commands(self):
        from piper_real.config import RealExperimentConfig
        from piper_real.recorder import CsvRecorder
        from piper_real.runner import RealExperimentRunner

        fake_sdk = FakeSdk()
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "excitation.csv"
            config = RealExperimentConfig(
                can_port="can0",
                control_mode="excitation_trajectory",
                control_rate_hz=100.0,
                record_rate_hz=100.0,
                output_csv=csv_path,
                duration=0.03,
                speed_rate=20,
                home_position=[0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
                position_amplitude=[0.05] * 6,
                position_frequency=[0.2] * 6,
                mit_kp=[10.0] * 6,
                mit_kd=[0.8] * 6,
                max_samples=2,
                trajectory_period=10.0,
                trajectory_harmonics=3,
                trajectory_coefficient_scale=0.2,
                trajectory_seed=7,
            )
            runner = RealExperimentRunner(
                config=config,
                adapter_factory=lambda cfg: __import__("piper_real.sdk_adapter", fromlist=["PiperSdkAdapter"]).PiperSdkAdapter(
                    sdk_factory=lambda can_port: fake_sdk,
                    can_port=cfg.can_port,
                ),
                recorder_factory=lambda path, dof: CsvRecorder(path, dof),
                sleep_fn=lambda _: None,
            )

            runner.run()

        self.assertTrue(any(call[0] == "JointMitCtrl" for call in fake_sdk.calls))
        self.assertTrue(any(call[:4] == ("MotionCtrl_2", 0x01, 0x04, 0) for call in fake_sdk.calls))


if __name__ == "__main__":
    unittest.main()
