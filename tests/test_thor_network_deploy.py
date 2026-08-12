import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_DIR = REPO_ROOT / "teleop" / "deploy" / "thor"
INSTALLER = DEPLOY_DIR / "install_thor_usb_ethernet.sh"


class ThorNetworkDeployTests(unittest.TestCase):
    def run_installer(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(INSTALLER), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_dry_run_contains_reproducible_networkmanager_settings(self) -> None:
        result = self.run_installer(
            "--dry-run",
            "--interface",
            "testnic0",
            "--connection",
            "test-thor-link",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(
            "+ nmcli connection add type ethernet ifname testnic0 "
            "con-name test-thor-link",
            result.stdout,
        )
        self.assertIn("ipv4.addresses 192.168.125.163/24", result.stdout)
        self.assertIn("connection.autoconnect yes", result.stdout)
        self.assertIn("ipv4.gateway ''", result.stdout)
        self.assertIn("ipv4.never-default yes", result.stdout)
        self.assertNotIn("uuid", result.stdout.lower())
        self.assertNotIn("mac-address", result.stdout.lower())

    def test_dry_run_uses_verified_default_interface(self) -> None:
        result = self.run_installer("--dry-run")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("enx80691a14d263", result.stdout)

    def test_invalid_interface_is_rejected_before_host_changes(self) -> None:
        result = self.run_installer("--dry-run", "--interface", "../../eth0")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported characters", result.stderr)
        self.assertNotIn("nmcli connection", result.stdout)

    def test_chrony_drop_ins_pin_source_and_client(self) -> None:
        source_lines = [
            line.strip()
            for line in (DEPLOY_DIR / "50-h2-source.conf").read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        allow_lines = [
            line.strip()
            for line in (DEPLOY_DIR / "60-h2-workstation.conf")
            .read_text()
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]

        self.assertEqual(source_lines, ["server 192.168.123.161 iburst"])
        self.assertEqual(allow_lines, ["allow 192.168.125.222/32"])


if __name__ == "__main__":
    unittest.main()
