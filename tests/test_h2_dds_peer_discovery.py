import json
import subprocess
import unittest
from unittest import mock

from teleop.utils.discover_h2_dds_peer import (
    H2PeerDiscoveryError,
    _parse_remote_results,
    discover_h2_body_peers_via_ssh,
)


def _result(process_name, peer):
    return json.dumps({"process_name": process_name, "peer": peer})


class H2DDSPeerDiscoveryTest(unittest.TestCase):
    def test_results_are_returned_in_required_process_order(self):
        output = "\n".join(
            [
                _result("humanoid", "192.168.123.161:36220"),
                _result("basic_service", "192.168.123.161:60363"),
            ]
        )

        peers = _parse_remote_results(
            output,
            ("basic_service", "humanoid"),
            "192.168.123.161",
        )

        self.assertEqual(
            peers,
            ["192.168.123.161:60363", "192.168.123.161:36220"],
        )

    def test_missing_process_fails_closed(self):
        with self.assertRaisesRegex(H2PeerDiscoveryError, "humanoid"):
            _parse_remote_results(
                _result("basic_service", "192.168.123.161:60363"),
                ("basic_service", "humanoid"),
                "192.168.123.161",
            )

    def test_wrong_source_address_fails_closed(self):
        output = "\n".join(
            [
                _result("basic_service", "192.168.123.161:60363"),
                _result("humanoid", "192.168.125.163:36220"),
            ]
        )
        with self.assertRaisesRegex(H2PeerDiscoveryError, "expected H2 address"):
            _parse_remote_results(
                output,
                ("basic_service", "humanoid"),
                "192.168.123.161",
            )

    @mock.patch("teleop.utils.discover_h2_dds_peer.subprocess.run")
    def test_single_ssh_call_runs_both_probes_concurrently(self, run):
        run.return_value = subprocess.CompletedProcess(
            args=[],
            returncode=0,
            stdout="\n".join(
                [
                    _result("basic_service", "192.168.123.161:60363"),
                    _result("humanoid", "192.168.123.161:36220"),
                ]
            ),
            stderr="",
        )

        peers = discover_h2_body_peers_via_ssh(
            ssh_target="unitree@thor",
            timeout_s=4,
        )

        self.assertEqual(
            peers,
            ["192.168.123.161:60363", "192.168.123.161:36220"],
        )
        run.assert_called_once()
        command = run.call_args.args[0]
        self.assertEqual(command[0], "ssh")
        self.assertIn("BatchMode=yes", command)
        self.assertIn("unitree@thor", command)
        remote_script = command[-1]
        self.assertIn("--process-name basic_service", remote_script)
        self.assertIn("--process-name humanoid", remote_script)
        self.assertIn(" & peer_probe_0=", remote_script)
        self.assertIn(" & peer_probe_1=", remote_script)

    @mock.patch("teleop.utils.discover_h2_dds_peer.subprocess.run")
    def test_nonzero_ssh_status_fails_closed(self, run):
        run.return_value = subprocess.CompletedProcess(
            args=[], returncode=1, stdout="", stderr="helper timed out"
        )

        with self.assertRaisesRegex(H2PeerDiscoveryError, "helper timed out"):
            discover_h2_body_peers_via_ssh(timeout_s=4)


if __name__ == "__main__":
    unittest.main()
