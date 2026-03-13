import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


class FakeBridgeAdapter:
    def __init__(self):
        self.connected = False
        self.enabled = False
        self.sent_commands = []
        self.state_index = 0
        self.states = [
            {
                "position": [0.0, 0.1, -0.1, 0.2, -0.2, 0.3],
                "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "effort": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
            },
            {
                "position": [0.01, 0.11, -0.09, 0.21, -0.19, 0.31],
                "velocity": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                "effort": [0.6, 0.7, 0.8, 0.9, 1.0, 1.1],
            },
        ]

    def connect(self):
        self.connected = True

    def enable(self, timeout_sec, sleep_fn):
        self.enabled = True

    def disconnect(self):
        self.connected = False

    def disable(self):
        self.enabled = False

    def emergency_stop(self):
        self.sent_commands.append(("emergency_stop",))

    def read_state(self):
        state = self.states[self.state_index]
        return type(
            "State",
            (),
            {
                "position": state["position"],
                "velocity": state["velocity"],
                "effort": state["effort"],
                "arm_status_code": 0,
                "ctrl_mode": 1,
                "mode_feed": 4,
                "has_error": False,
            },
        )()

    def send_mit_command(self, position, velocity, kp, kd, torque):
        self.sent_commands.append(("mit", position, velocity, kp, kd, torque))
        self.state_index = 1

    def send_joint_position(self, position, speed_rate):
        self.sent_commands.append(("joint_position", position, speed_rate))
        self.state_index = min(self.state_index + 1, len(self.states) - 1)


class BridgeSessionTest(unittest.TestCase):
    def test_init_command_connects_and_returns_initial_state(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=False,
            joint_soft_limits=[(-1.0, 1.0)] * 6,
            max_command_velocity=[2.0] * 6,
            max_command_torque=[5.0] * 6,
        )

        response = session.handle_command("INIT")

        self.assertEqual(response.split()[0], "STATE")
        self.assertTrue(adapter.connected)
        self.assertTrue(adapter.enabled)
        self.assertIn("0.100000", response)
        self.assertIn("0.500000", response)

    def test_init_moves_robot_to_home_before_allowing_experiment(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        adapter.states = [
            {
                "position": [0.4, 0.4, -0.4, 0.4, -0.4, 0.4],
                "velocity": [0.0] * 6,
                "effort": [0.0] * 6,
            },
            {
                "position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "velocity": [0.0] * 6,
                "effort": [0.0] * 6,
            },
        ]
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=True,
            home_position=[0.0] * 6,
            home_position_tolerance=[0.05] * 6,
            home_speed_rate=15,
            joint_soft_limits=[(-1.0, 1.0)] * 6,
            max_command_velocity=[2.0] * 6,
            max_command_torque=[5.0] * 6,
        )

        response = session.handle_command("INIT")

        self.assertEqual(adapter.sent_commands[0], ("joint_position", [0.0] * 6, 15))
        self.assertEqual(response.split()[0], "STATE")
        self.assertIn("0.000000", response)

    def test_step_command_sends_unified_mit_payload_and_returns_next_state(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=False,
            joint_soft_limits=[(-1.0, 1.0)] * 6,
            max_command_velocity=[2.0] * 6,
            max_command_torque=[5.0] * 6,
        )
        session.handle_command("INIT")

        command = "STEP " + " ".join(
            str(value)
            for value in (
                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
                + [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
                + [10.0] * 6
                + [0.8] * 6
                + [1.0] * 6
            )
        )

        response = session.handle_command(command)

        self.assertEqual(response.split()[0], "STATE")
        self.assertEqual(adapter.sent_commands[0][0], "mit")
        self.assertIn("0.310000", response)

    def test_step_rejects_out_of_limit_position_command(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=False,
            joint_soft_limits=[(-0.2, 0.2)] * 6,
            max_command_velocity=[2.0] * 6,
            max_command_torque=[5.0] * 6,
        )
        session.handle_command("INIT")

        command = "STEP " + " ".join(["0.3"] * 30)
        response = session.handle_command(command)

        self.assertTrue(response.startswith("ERROR command_position_limit"))
        self.assertEqual(adapter.sent_commands[-1], ("emergency_stop",))

    def test_step_rejects_out_of_limit_velocity_command(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=False,
            joint_soft_limits=[(-1.0, 1.0)] * 6,
            max_command_velocity=[0.5] * 6,
            max_command_torque=[5.0] * 6,
        )
        session.handle_command("INIT")

        payload = (
            [0.0] * 6
            + [0.8] * 6
            + [10.0] * 6
            + [0.8] * 6
            + [1.0] * 6
        )
        response = session.handle_command("STEP " + " ".join(map(str, payload)))

        self.assertTrue(response.startswith("ERROR command_velocity_limit"))
        self.assertEqual(adapter.sent_commands[-1], ("emergency_stop",))

    def test_step_rejects_out_of_limit_torque_command(self):
        from piper_real.bridge import BridgeSession

        adapter = FakeBridgeAdapter()
        session = BridgeSession(
            adapter=adapter,
            sleep_fn=lambda _: None,
            move_to_home_before_start=False,
            joint_soft_limits=[(-1.0, 1.0)] * 6,
            max_command_velocity=[2.0] * 6,
            max_command_torque=[2.0] * 6,
        )
        session.handle_command("INIT")

        payload = (
            [0.0] * 6
            + [0.0] * 6
            + [10.0] * 6
            + [0.8] * 6
            + [3.0] * 6
        )
        response = session.handle_command("STEP " + " ".join(map(str, payload)))

        self.assertTrue(response.startswith("ERROR command_torque_limit"))
        self.assertEqual(adapter.sent_commands[-1], ("emergency_stop",))


if __name__ == "__main__":
    unittest.main()
