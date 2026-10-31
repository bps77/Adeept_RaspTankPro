import ast
from pathlib import Path
import unittest
from unittest.mock import Mock


class TestWifiCheck(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / "server" / "webServer.py"
        self.tree = ast.parse(source.read_text(), filename=str(source))
        wifi_check = next(
            node for node in self.tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "wifi_check"
        )
        self.socket = Mock()
        self.socket.socket.return_value.getsockname.return_value = ("192.0.2.1", 1234)
        self.screen = Mock()
        self.light = Mock()
        self.threading = Mock()
        self.namespace = {
            "socket": self.socket,
            "screen": self.screen,
            "OLED_connection": True,
            "RL": self.light,
            "threading": self.threading,
            "ap_thread": Mock(),
            "time": Mock(),
            "print": Mock(),
        }
        # Load only the function to avoid the server's import-time hardware startup.
        module = ast.Module(body=[wifi_check], type_ignores=[])
        exec(compile(module, str(source), "exec"), self.namespace)

    def test_connected_wifi_displays_address_without_updating_code(self):
        self.namespace["wifi_check"]()

        self.socket.socket.return_value.connect.assert_called_once_with(("1.1.1.1", 80))
        self.socket.socket.return_value.close.assert_called_once_with()
        self.screen.screen_show.assert_any_call(2, "IP:192.0.2.1")
        self.screen.screen_show.assert_any_call(3, "AP MODE OFF")
        self.threading.Thread.assert_not_called()
        self.light.pause.assert_not_called()

    def test_connected_wifi_without_oled(self):
        self.namespace["OLED_connection"] = False

        self.namespace["wifi_check"]()

        self.socket.socket.return_value.close.assert_called_once_with()
        self.screen.screen_show.assert_not_called()
        self.threading.Thread.assert_not_called()

    def test_disconnected_wifi_starts_access_point(self):
        self.socket.socket.return_value.connect.side_effect = OSError("No network")

        self.namespace["wifi_check"]()

        self.light.pause.assert_called_once_with()
        self.threading.Thread.assert_called_once_with(target=self.namespace["ap_thread"])
        self.threading.Thread.return_value.start.assert_called_once_with()
        self.screen.screen_show.assert_any_call(2, "IP:192.168.12.1")
        self.screen.screen_show.assert_any_call(3, "AP MODE ON")

    def test_updater_function_is_removed(self):
        self.assertFalse(any(
            isinstance(node, ast.FunctionDef) and node.name == "update_code"
            for node in self.tree.body
        ))


if __name__ == "__main__":
    unittest.main()
