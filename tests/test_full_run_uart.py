"""Read-only telemetry selection on the two already-owned MCU UARTs."""
import unittest

from jetson.amr_core.full_run_legacy import LegacyControlAdapter
from jetson.amr_core.full_run_uart import StatusReceiver


def status(base=0):
    return f"$STATUS,{base},1234,45,0,0,0,0,0,0,0\r\n".encode()


def control(session=123):
    return f"$CTRL,1,{session},0,0,0,0,0,0,0,0,1000\r\n".encode()


class ReadOnlyUart:
    # No write() method: the receiver must never send a probe/command.
    def __init__(self):
        self.incoming = bytearray()
        self.resets = 0
        self.reads = []

    @property
    def in_waiting(self):
        return len(self.incoming)

    def reset_input_buffer(self):
        self.incoming.clear()
        self.resets += 1

    def read(self, count):
        self.reads.append(count)
        result = bytes(self.incoming[:count])
        del self.incoming[:count]
        return result


class StatusReceiverTests(unittest.TestCase):
    def setUp(self):
        self.command = ReadOnlyUart()
        self.telemetry = ReadOnlyUart()
        self.events = []

    def receiver(self, protocol="legacy", enabled=True, same_uart=False):
        return StatusReceiver(self.command, self.command if same_uart else self.telemetry,
                              "commands", "status", protocol, 123,
                              command_fallback=enabled,
                              emit=lambda event, **data: self.events.append((event, data)))

    def test_no_bytes_is_distinguished_from_invalid_frames(self):
        receiver = self.receiver()
        self.assertEqual(receiver.poll(1), ([], False))
        health = receiver.snapshot(2)
        self.assertIsNone(health["active_status_port"])
        self.assertTrue(health["diagnosis"].startswith("no_RX_bytes_"))
        self.assertTrue(health["zero_TX_is_not_proof_MCU_received"])
        for port in ("commands", "status"):
            self.assertEqual(health["ports"][port]["rx_bytes"], 0)
            self.assertIsNone(health["ports"][port]["valid_age_s"])

    def test_received_robot_compact_format_is_selected_with_confirmed_field_mapping(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"$STATUS,0,0,0,-1,-1,80\r\n" * 5)
        lines, overflow = receiver.poll(10)
        self.assertEqual(len(lines), 5)
        self.assertFalse(overflow)
        self.assertEqual(receiver.active_port, "commands")
        self.assertIsNone(receiver.unsupported_layout())
        self.assertEqual(receiver.stats["commands"]["valid_frames"], 5)

    def test_truly_unknown_layout_is_still_diagnosed_not_decoded_as_zero_feedback(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"$STATUS,0,0,0,-1,-1,80,0\r\n" * 5)
        self.assertEqual(receiver.poll(10), ([], False))
        mismatch = receiver.unsupported_layout()
        self.assertEqual(mismatch["field_count"], 8)
        self.assertEqual(mismatch["expected_field_counts"], [7, 11])
        self.assertIsNone(receiver.active_port)
        self.assertIn("deployed_schema", receiver.snapshot(10)["diagnosis"])

    def test_one_short_frame_does_not_overrule_a_later_valid_stream(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"$STATUS,0,0,0,-1,-1,80,0\n")
        receiver.poll(10)
        self.assertIsNone(receiver.unsupported_layout())
        self.command.incoming.extend(status())
        receiver.poll(11)
        self.assertEqual(receiver.active_port, "commands")
        self.assertIsNone(receiver.unsupported_layout())

    def test_valid_other_port_wins_over_repeated_unknown_schema(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"$STATUS,30,0,0,-1,-1,80,0\n" * 6)
        self.telemetry.incoming.extend(status())
        lines, _ = receiver.poll(10)
        self.assertEqual(lines, [status().decode().strip()])
        self.assertEqual(receiver.active_port, "status")
        self.assertIsNone(receiver.unsupported_layout())

    def test_corrupt_noninteger_short_frames_are_not_misidentified_as_a_schema(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"$STATUS,0,bad,0,-1,-1,80\n" * 8)
        receiver.poll(10)
        self.assertIsNone(receiver.unsupported_layout())

    def test_configured_status_wins_when_both_sources_have_valid_frames(self):
        receiver = self.receiver()
        self.telemetry.incoming.extend(status(10))
        self.command.incoming.extend(status(50))
        lines, overflow = receiver.poll(10)
        self.assertEqual(lines, [status(10).decode().strip()])
        self.assertFalse(overflow)
        self.assertEqual(receiver.active_port, "status")
        selected = [data for event, data in self.events if event == "uart_status_source"]
        self.assertEqual(len(selected), 1)
        self.assertFalse(selected[0]["fallback"])

    def test_command_RX_is_selected_only_after_a_valid_frame(self):
        receiver = self.receiver()
        self.command.incoming.extend(b"boot ready\r\n" + status(4))
        self.assertEqual(receiver.poll(10), ([], False))
        self.assertIsNone(receiver.active_port)
        self.command.incoming.extend(status(5))
        self.assertEqual(receiver.poll(11)[0], [status(5).decode().strip()])
        self.assertEqual(receiver.active_port, "commands")
        stats = receiver.snapshot(12)["ports"]["commands"]
        self.assertEqual(stats["invalid_frames"], 1)
        self.assertEqual(stats["valid_frames"], 1)
        self.assertEqual(stats["valid_age_s"], 1)

    def test_disabled_command_fallback_does_not_read_command_UART(self):
        receiver = self.receiver(enabled=False)
        self.command.incoming.extend(status())
        self.assertEqual(receiver.poll(10), ([], False))
        self.assertEqual(self.command.reads, [])
        self.assertEqual(self.command.resets, 0)
        self.assertEqual(set(receiver.snapshot(10)["ports"]), {"status"})

    def test_shared_handle_is_reset_and_read_only_once(self):
        receiver = self.receiver(same_uart=True)
        self.command.incoming.extend(status())
        self.assertEqual(receiver.poll(10)[0], [status().decode().strip()])
        self.assertEqual(self.command.resets, 1)
        self.assertEqual(len(self.command.reads), 1)
        self.assertEqual(set(receiver.snapshot(10)["ports"]), {"status"})

    def test_partial_lines_from_different_ports_never_get_joined(self):
        receiver = self.receiver()
        frame = status(10)
        self.telemetry.incoming.extend(frame[:12])
        self.command.incoming.extend(frame[12:])
        self.assertEqual(receiver.poll(10), ([], False))
        self.assertIsNone(receiver.active_port)
        self.telemetry.incoming.extend(frame[12:])
        self.assertEqual(receiver.poll(11)[0], [frame.decode().strip()])
        self.assertEqual(receiver.active_port, "status")

    def test_selected_source_never_switches_to_another_port(self):
        receiver = self.receiver()
        self.command.incoming.extend(status(5))
        receiver.poll(10)
        self.telemetry.incoming.extend(status(20))
        self.assertEqual(receiver.poll(20), ([], False))
        self.assertEqual(receiver.active_port, "commands")
        self.assertEqual(receiver.snapshot(20)["ports"]["commands"]["valid_age_s"], 10)
        self.assertEqual(sum(event == "uart_status_source" for event, _ in self.events), 1)

    def test_standby_status_does_not_apply_duplicate_or_spurious_buttons(self):
        receiver = self.receiver()
        adapter = LegacyControlAdapter(123)
        self.telemetry.incoming.extend(status(0))
        self.command.incoming.extend(status(40))
        for line in receiver.poll(10)[0]:
            adapter.decode(line, 10)
        self.telemetry.incoming.extend(status(5))
        self.command.incoming.extend(status(65))
        for line in receiver.poll(11)[0]:
            adapter.decode(line, 11)
        self.assertEqual(adapter.selected_rpm, 5)
        self.assertEqual(adapter.up_count, 1)

    def test_wrong_protocol_is_reported_without_selecting_it(self):
        receiver = self.receiver()
        self.telemetry.incoming.extend(control())
        self.assertEqual(receiver.poll(10), ([], False))
        health = receiver.snapshot(10)
        self.assertEqual(health["diagnosis"], "received_other_MCU_protocol_check_MCU_PROTOCOL")
        self.assertEqual(health["ports"]["status"]["other_protocol_frames"], 1)

    def test_CTRL_requires_the_current_session(self):
        receiver = self.receiver(protocol="ctrl")
        self.telemetry.incoming.extend(control(session=124))
        self.assertEqual(receiver.poll(10), ([], False))
        self.assertIsNone(receiver.active_port)
        self.command.incoming.extend(control())
        self.assertEqual(receiver.poll(11)[0], [control().decode().strip()])
        self.assertEqual(receiver.active_port, "commands")
        # Once selected, a changed session must reach the normal parser/core:
        # that is a lost/rebooted MCU, not a chance to select another source.
        self.command.incoming.extend(control(session=125))
        self.assertEqual(receiver.poll(12)[0], [control(125).decode().strip()])

    def test_invalid_selected_frame_is_forwarded_to_fail_safe_parser(self):
        receiver = self.receiver()
        self.telemetry.incoming.extend(status())
        receiver.poll(10)
        self.telemetry.incoming.extend(status(66))
        self.assertEqual(receiver.poll(11)[0], [status(66).decode().strip()])
        self.assertEqual(receiver.stats["status"]["invalid_frames"], 1)

    def test_invalid_status_ranges_never_select_a_source(self):
        for frame in (status(66), status(4), b"$STATUS,0,99999,45,0,0,0,0,0,0,0\n",
                      b"$STATUS,0,1234,45,0,0,1001,0,0,0,0\n",
                      b"$STATUS,0,1234,45,0,0,0,0,0,0,2\n",
                      b"$STATUS,bad\n"):
            with self.subTest(frame=frame):
                receiver = self.receiver()
                self.telemetry.incoming.extend(frame)
                self.assertEqual(receiver.poll(10), ([], False))
                self.assertIsNone(receiver.active_port)
                self.assertIn("invalid_MCU_frames", receiver.snapshot(10)["diagnosis"])

    def test_noise_without_lines_reports_bytes_and_bounded_buffer(self):
        receiver = self.receiver()
        self.telemetry.incoming.extend(b"x" * 9000)
        for now in range(3):
            self.assertEqual(receiver.poll(now), ([], False))
            self.assertLessEqual(len(receiver.buffers["status"]), 8192)
        health = receiver.snapshot(3)
        self.assertEqual(health["ports"]["status"]["rx_bytes"], 9000)
        self.assertEqual(health["ports"]["status"]["overflows"], 1)
        self.assertIn("bytes_without_valid_status", health["diagnosis"])
        self.assertTrue(all(size <= 4096 for size in self.telemetry.reads))

    def test_selected_source_overflow_invalidates_freshness_then_recovers(self):
        receiver = self.receiver()
        self.telemetry.incoming.extend(status())
        receiver.poll(10)
        self.telemetry.incoming.extend(b"x" * 8193)
        self.assertEqual(receiver.poll(11), ([], False))
        self.assertEqual(receiver.poll(12), ([], False))
        self.assertEqual(receiver.poll(13), ([], True))
        self.telemetry.incoming.extend(status(5))
        self.assertEqual(receiver.poll(14), ([status(5).decode().strip()], False))

    def test_old_buffered_status_is_flushed_at_startup(self):
        self.command.incoming.extend(status(65))
        self.telemetry.incoming.extend(status(65))
        receiver = self.receiver()
        self.assertEqual(receiver.poll(10), ([], False))
        self.assertIsNone(receiver.active_port)


if __name__ == "__main__":
    unittest.main()
