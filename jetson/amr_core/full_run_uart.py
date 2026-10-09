"""Bounded RX discovery on the two already-owned MCU UARTs; never writes."""
from .full_run import ControlStatus
from .full_run_legacy import UnsupportedLegacyStatus, validate_legacy_status


class StatusReceiver:
    def __init__(self, command_uart, status_uart, command_port, status_port,
                 protocol, session, *, command_fallback=True, emit=None):
        self.protocol, self.session = protocol, session
        self.expected = "$STATUS" if protocol == "legacy" else "$CTRL"
        self.emit = emit or (lambda *_args, **_kwargs: None)
        self.active_port = None
        self.sources = [(status_port, status_uart)]
        if command_fallback and command_uart is not status_uart:
            self.sources.append((command_port, command_uart))
        self.buffers = {port: bytearray() for port, _ in self.sources}
        self.stats = {port: {"rx_bytes": 0, "chunks": 0, "lines": 0, "valid_frames": 0,
                             "invalid_frames": 0, "overflows": 0, "other_protocol_frames": 0,
                             "last_rx_s": None, "last_valid_s": None, "last_line": None}
                      for port, _ in self.sources}
        self.schemas = {port: {} for port, _ in self.sources}
        for port, uart in self.sources:
            uart.reset_input_buffer()
            self.emit("uart_open", port=port, role="configured_status" if port == status_port else "command_RX_fallback",
                      settings={key: getattr(uart, key, None) for key in
                                ("baudrate", "bytesize", "parity", "stopbits", "timeout",
                                 "write_timeout", "xonxoff", "rtscts", "dsrdtr")})

    def poll(self, now):
        batches, valid_sources, overflows = {}, set(), set()
        for port, uart in self.sources:
            stats, buffer = self.stats[port], self.buffers[port]
            incoming = uart.read(min(uart.in_waiting, 4096))
            if incoming:
                stats["rx_bytes"] += len(incoming)
                stats["chunks"] += 1
                stats["last_rx_s"] = now
                self.emit("uart_rx_chunk", port=port, hex=incoming.hex(),
                          text=incoming.decode("ascii", errors="replace"))
                buffer.extend(incoming)
            if len(buffer) > 8192:
                stats["overflows"] += 1
                overflows.add(port)
                self.emit("uart_overflow", port=port, bytes=len(buffer))
                buffer.clear()
            lines = []
            while b"\n" in buffer:
                line, _, rest = buffer.partition(b"\n")
                buffer[:] = rest
                decoded = line.rstrip(b"\r").decode("ascii", errors="replace")
                stats["lines"] += 1
                stats["last_line"] = decoded[:256]
                self.emit("uart_line", port=port, line=decoded)
                lines.append(decoded)
                if decoded.startswith(self.expected):
                    try:
                        parsed = (validate_legacy_status(decoded) if self.protocol == "legacy"
                                  else ControlStatus.decode(decoded))
                        if self.protocol == "ctrl" and parsed.session != self.session:
                            raise ValueError("CTRL session does not match this run")
                        stats["valid_frames"] += 1
                        stats["last_valid_s"] = now
                        valid_sources.add(port)
                    except ValueError as exc:
                        stats["invalid_frames"] += 1
                        self.emit("uart_candidate_invalid", port=port, line=decoded, error=str(exc))
                        if isinstance(exc, UnsupportedLegacyStatus):
                            values = decoded.strip().split(",")[1:]
                            # Repeated complete integer-shaped frames indicate
                            # a different layout, not a partial serial read. Do
                            # not guess what the numbers mean or decode them as
                            # wheel/button/stop data.
                            numeric = False
                            if all(len(v) <= 64 for v in values):
                                try:
                                    [int(v) for v in values]
                                    numeric = True
                                except ValueError:
                                    pass
                            key = str(exc.received_fields)
                            if (numeric and values
                                    and (key in self.schemas[port] or len(self.schemas[port]) < 32)):
                                schema = self.schemas[port].setdefault(key, {
                                    "field_count": exc.received_fields, "frames": 0,
                                    "first_seen_s": now, "last_seen_s": now, "raw_values": None})
                                schema["frames"] += 1
                                schema["last_seen_s"], schema["raw_values"] = now, values
                elif decoded.startswith(("$STATUS", "$CTRL")):
                    stats["other_protocol_frames"] += 1
            batches[port] = lines
        if self.active_port is None:
            # Prefer the configured status UART if both have a valid frame in
            # this cycle. No button events are applied during validation.
            for port, _ in self.sources:
                if port in valid_sources:
                    self.active_port = port
                    self.emit("uart_status_source", port=port, expected=self.expected,
                              fallback=port != self.sources[0][0],
                              message="Validated RX source locked until this process exits")
                    break
        # Never switch sources after selection: a lost link must still fault,
        # not be hidden by unrelated/stale traffic on a different UART.
        return batches.get(self.active_port, []), self.active_port in overflows

    def unsupported_layout(self, minimum_frames=5):
        if self.active_port is not None or self.protocol != "legacy":
            return None
        for port, _ in self.sources:
            for schema in self.schemas[port].values():
                if schema["frames"] >= minimum_frames:
                    return {"port": port, **schema, "expected_field_counts": [7, 11],
                            "values_semantics": "unknown_requires_actual_STM32_STATUS_transmit_definition"}
        return None

    def snapshot(self, now):
        ports = {}
        for port, values in self.stats.items():
            ports[port] = dict(values, buffered_bytes=len(self.buffers[port]))
            ports[port]["unsupported_layouts"] = self.schemas[port]
            ports[port]["rx_age_s"] = None if values["last_rx_s"] is None else now - values["last_rx_s"]
            ports[port]["valid_age_s"] = None if values["last_valid_s"] is None else now - values["last_valid_s"]
        if self.active_port:
            diagnosis = "validated_status_source_selected"
        elif not any(v["rx_bytes"] for v in self.stats.values()):
            diagnosis = "no_RX_bytes_verify_STM32_TX_to_Jetson_RX_port_wiring_common_GND_and_MCU_power"
        elif self.unsupported_layout():
            diagnosis = "MCU_STATUS_received_but_deployed_schema_does_not_match_supported_7_or_11_fields"
        elif any(v["other_protocol_frames"] for v in self.stats.values()):
            diagnosis = "received_other_MCU_protocol_check_MCU_PROTOCOL"
        elif any(v["invalid_frames"] for v in self.stats.values()):
            diagnosis = "received_invalid_MCU_frames_check_format_fields_and_baudrate"
        else:
            diagnosis = "received_bytes_without_valid_status_check_baudrate_line_endings_or_wrong_signal"
        return {"active_status_port": self.active_port, "expected": self.expected,
                "diagnosis": diagnosis, "ports": ports,
                "zero_TX_is_not_proof_MCU_received": True}
