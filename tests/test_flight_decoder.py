import importlib.util
import struct
import tempfile
import unittest
from pathlib import Path

script = Path(__file__).parents[1] / 'tools' / '解码飞行记录.py'
spec = importlib.util.spec_from_file_location('decoder', script)
decoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)


class DecoderTests(unittest.TestCase):
    def fixture(self, payload=b''):
        header = b'EIRFLT01' + struct.pack('<IIqq', 1, 64, 1, 2) + bytes(32)
        body = struct.pack('<HHIqqqQQI', 0, 0, 0, 3, 4, 5, 6, 7, len(payload)) + payload
        return header + struct.pack('<II', len(body), decoder.checksum(body)) + body

    def read(self, data):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'record.bin'
            file.write_bytes(data)
            issues = []
            rows = list(decoder.records(file, issues))
            return rows, issues

    def test_golden_double_and_integer_contract(self):
        payload = struct.pack('<HHId', 20, 1, 1, -3.25) + struct.pack('<HHII', 80, 2, 1, 123)
        rows, issues = self.read(self.fixture(payload))
        self.assertEqual(issues, [])
        self.assertEqual(rows[0]['input_q.0'], -3.25)
        self.assertEqual(rows[0]['component.0'], 123)
        self.assertEqual(rows[0]['source_ns'], 5)

    def test_application_uint64_identity_is_not_rounded(self):
        exact = (1 << 63) + 123
        values = [exact & 0xffffffff, exact >> 32] + [0] * 22
        payload = struct.pack('<HHI24I', 512, 2, 24, *values)
        rows, issues = self.read(self.fixture(payload))
        self.assertEqual(issues, [])
        self.assertEqual(rows[0]['session'], exact)
        self.assertEqual(rows[0]['app_identity_u32.0'], values[0])

    def test_unknown_application_tags_still_decode(self):
        payload = struct.pack('<HHII', 799, 2, 1, 42)
        rows, issues = self.read(self.fixture(payload))
        self.assertEqual(issues, [])
        self.assertEqual(rows[0]['field_799.0'], 42)

    def test_extended_identity_keeps_control_and_io_feedback_separate(self):
        values = [0] * 28
        values[2:6] = [17, 0, 123, 0]
        values[24:28] = [19, 0, 456, 0]
        rows, issues = self.read(self.fixture(struct.pack('<HHI28I', 512, 2, 28, *values)))
        self.assertEqual(issues, [])
        self.assertEqual(rows[0]['feedback_sequence'], 17)
        self.assertEqual(rows[0]['feedback_stamp_ns'], 123)
        self.assertEqual(rows[0]['io_feedback_sequence'], 19)
        self.assertEqual(rows[0]['io_feedback_stamp_ns'], 456)

    def test_truncated_tail_keeps_complete_records(self):
        rows, issues = self.read(self.fixture() + self.fixture()[64:-3])
        self.assertEqual(len(rows), 1)
        self.assertEqual(issues, ['truncated record body'])

    def test_corrupt_record_does_not_hide_following_good_record(self):
        broken = bytearray(self.fixture())
        broken[-5] ^= 1
        rows, issues = self.read(broken + self.fixture()[64:])
        self.assertEqual(len(rows), 1)
        self.assertEqual(issues, ['checksum mismatch'])

    def test_unsupported_version_rejected(self):
        data = bytearray(self.fixture())
        data[8] = 2
        with self.assertRaises(ValueError):
            self.read(data)

    def test_bad_field_length_rejected(self):
        _, issues = self.read(self.fixture(struct.pack('<HHI', 20, 1, 20)))
        self.assertEqual(issues, ['invalid field payload'])


if __name__ == '__main__':
    unittest.main()
