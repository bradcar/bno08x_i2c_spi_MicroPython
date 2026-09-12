"""Offline SH-2 timestamp contract tests; no device import or initialization.

Run with CPython: python tests/test_timestamps.py
Optional --source points to a trusted earlier lib/bno08x.py for comparison.
The actual three parser method bodies are compiled without native decorators;
only transport, the no-wrap host clock and uctypes field access are replaced.
"""

import argparse
import ast
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "lib" / "bno08x.py"
BASE, REBASE, ACCEL, ROTATION = 0xFB, 0xFA, 0x01, 0x05


def load_parsers(source):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "BNO08X")
    names = {"update_sensors", "_process_report", "_process_control_report"}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {m.name for m in methods} == names
    for method in methods:
        method.decorator_list = []

    def report_fields(data, layout, endian):
        values = struct.unpack_from("<hhhh", data, 4) if data[0] == ROTATION else (
            *struct.unpack_from("<hhh", data, 4), 0)
        return SimpleNamespace(byte2=data[2], byte3=data[3],
                               **dict(zip(("v1", "v2", "v3", "v4"), values)))

    env = {
        "ticks_diff": lambda end, start: end - start,
        "unpack_from": struct.unpack_from,
        "_BASE_TIMESTAMP": BASE,
        "_TIMESTAMP_REBASE": REBASE,
        "_REPORT_LENGTHS": {BASE: 5, REBASE: 5, ACCEL: 10, ROTATION: 14},
        "_SENSOR_SCALING": {ACCEL: (2 ** -8, 3), ROTATION: (2 ** -14, 4)},
        "_SENSOR_REPORT_LAYOUT": None,
        "uctypes": SimpleNamespace(addressof=lambda data: data, struct=report_fields,
                                    LITTLE_ENDIAN=0),
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), env)
    return env


def delta(report_id, ticks):
    return struct.pack("<Bi", report_id, ticks)


def sensor(report_id=ROTATION, delay_ticks=0):
    assert 0 <= delay_ticks <= 0x3FFF
    header = bytes((report_id, 0, ((delay_ticks >> 8) << 2) | 3, delay_ticks & 255))
    if report_id == ROTATION:
        return header + struct.pack("<hhhhH", 0, 0, 0, 16384, 0)
    return header + struct.pack("<hhh", 256, -512, 768)


def driver(parsers):
    obj = SimpleNamespace(_new_data_interrupt=True, _report_values={},
                          _unread_report_count={ACCEL: 0, ROTATION: 0},
                          ms_at_interrupt=5000, _epoch_start_ms=0,
                          _last_base_timestamp_us=0)
    for name in ("_process_report", "_process_control_report"):
        setattr(obj, name, lambda report_id, data, method=name:
                parsers[method](obj, report_id, data))
    return obj


class TimestampContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parsers = load_parsers(SOURCE)

    def replay(self, records, path="fast"):
        obj = driver(self.parsers)
        if path == "fast":
            payload = b"".join(records)
            obj._read_packet = lambda wait: (payload, 3, len(payload))
            self.assertEqual(self.parsers["update_sensors"](obj), 1)
        else:
            for record in records:
                # Control parser supports timestamp records, not sensor values.
                method = (obj._process_control_report if path == "control"
                          and record[0] in (BASE, REBASE) else obj._process_report)
                method(record[0], record)
        return obj

    def assert_timestamp(self, records, expected):
        for path in ("fast", "report", "control"):
            with self.subTest(path=path):
                result = self.replay(records, path)._report_values[ROTATION]
                self.assertAlmostEqual(result[-1], expected)
                self.assertEqual(result[:-1], (1, 0, 0, 0, 3))

    def test_zero_base_control(self):
        self.assert_timestamp([delta(BASE, 0), sensor()], 5000)

    def test_positive_base_and_report_delay_control(self):
        self.assert_timestamp([delta(BASE, 40000), sensor(delay_ticks=10000)], 2000)

    def test_negative_base_is_signed(self):
        self.assert_timestamp([delta(BASE, -10), sensor()], 5001)

    def test_manufacturer_rebase_example(self):
        self.assert_timestamp([delta(BASE, 40000), delta(REBASE, 15000),
                               sensor(delay_ticks=10000)], 3500)

    def test_negative_rebase_is_signed(self):
        self.assert_timestamp([delta(BASE, 10000), delta(REBASE, -2500),
                               sensor(delay_ticks=500)], 3800)

    def test_multiple_rebases_accumulate(self):
        self.assert_timestamp([delta(BASE, 40000), delta(REBASE, 15000),
                               delta(REBASE, -1000), delta(REBASE, 500),
                               sensor(delay_ticks=10000)], 3450)

    def test_new_base_replaces_previous_rebases(self):
        self.assert_timestamp([delta(BASE, 40000), delta(REBASE, 15000),
                               delta(BASE, 100), sensor(delay_ticks=50)], 4995)

    def test_rebase_changes_only_following_reports(self):
        records = [delta(BASE, 40000), sensor(ACCEL), delta(REBASE, 15000),
                   sensor(delay_ticks=10000)]
        for path in ("fast", "report", "control"):
            with self.subTest(path=path):
                obj = self.replay(records, path)
                self.assertEqual(obj._report_values[ACCEL], (1, -2, 3, 3, 1000))
                self.assertEqual(obj._report_values[ROTATION][-1], 3500)
                self.assertEqual(obj._unread_report_count, {ACCEL: 1, ROTATION: 1})

    def test_next_packet_base_does_not_inherit_prior_rebase(self):
        obj = driver(self.parsers)
        packets = [b"".join([delta(BASE, 40000), delta(REBASE, 15000), sensor()]),
                   b"".join([delta(BASE, 100), sensor(delay_ticks=50)])]

        def read_packet(wait):
            payload = packets.pop(0)
            obj._new_data_interrupt = bool(packets)
            return payload, 3, len(payload)

        obj._read_packet = read_packet
        self.assertEqual(self.parsers["update_sensors"](obj), 2)
        self.assertEqual(obj._report_values[ROTATION][-1], 4995)
        self.assertEqual(obj._unread_report_count[ROTATION], 2)

    def test_interrupt_during_rebase_keeps_current_packet_anchor(self):
        obj = driver(self.parsers)
        payload = b"".join([delta(BASE, 40000), sensor(ACCEL),
                            delta(REBASE, 15000), sensor(delay_ticks=10000)])
        reads = iter([(payload, 3, len(payload)), None])
        obj._read_packet = lambda wait: next(reads)
        process_report = obj._process_report

        def interrupt_during_rebase(report_id, data):
            if report_id == REBASE:
                # A following IRQ must not retimestamp the packet in progress.
                obj.ms_at_interrupt = 5100
                obj._new_data_interrupt = True
            process_report(report_id, data)

        obj._process_report = interrupt_during_rebase
        self.assertEqual(self.parsers["update_sensors"](obj), 1)
        self.assertEqual(obj._report_values[ACCEL][-1], 1000)
        self.assertEqual(obj._report_values[ROTATION][-1], 3500)
        self.assertEqual(obj._unread_report_count, {ACCEL: 1, ROTATION: 1})
        self.assertEqual(obj.ms_at_interrupt, 5100)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    args = parser.parse_args()
    SOURCE = args.source
    unittest.main(argv=[__file__], verbosity=2)
