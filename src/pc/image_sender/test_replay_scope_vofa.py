import csv
import struct
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

import replay_scope_vofa as vofa


class FakeSocket:
    def __init__(self, incoming=b''):
        self.incoming = bytearray(incoming)

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def recv(self, size):
        block = bytes(self.incoming[:size])
        del self.incoming[:size]
        return block


class FakeClient:
    def __init__(self):
        self.sent = bytearray()
        self.reads = 0

    def recv(self, _size):
        self.reads += 1
        return b'' if self.reads > 1 else b'connected'

    def sendall(self, data):
        self.sent.extend(data)


def cap3_record(points=256):
    header = bytearray(40)
    header[:8] = b'V32CAP3!'
    struct.pack_into('<I', header, 8, 3)
    struct.pack_into('<I', header, 24, points)
    samples = b''.join(struct.pack('<hh', i, -i) for i in range(points))
    return bytes(header) + samples + struct.pack('<I', zlib.crc32(header + samples))


class FireWaterOutputTests(unittest.TestCase):
    def assert_two_numeric_columns(self, line):
        self.assertFalse(line.startswith('wave:'))
        values = line.strip().split(',')
        self.assertEqual(len(values), 2)
        for value in values:
            float(value)

    def test_sample_format_has_two_numeric_columns(self):
        self.assertEqual(vofa.firewater_sample(12, -34), '-12,34\n')
        self.assertEqual(vofa.firewater_sample(12, -34, (.5, .25)), '-6,8.5\n')
        self.assert_two_numeric_columns(vofa.firewater_sample(12, -34))
        self.assert_two_numeric_columns(vofa.firewater_sample(12, -34, (.5, .25)))

    def test_csv_export_has_two_numeric_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'scope.csv'
            with path.open('w', newline='', encoding='utf-8') as file:
                writer = csv.DictWriter(file, fieldnames=('sample_index', 'ch1_adc_code', 'ch2_adc_code'))
                writer.writeheader()
                writer.writerow({'sample_index': 0, 'ch1_adc_code': 12, 'ch2_adc_code': -34})
            lines = list(vofa.firewater(path))
        self.assertEqual(lines, [b'-12,34\n'])
        self.assert_two_numeric_columns(lines[0].decode('ascii'))

    def test_live_export_has_two_numeric_columns_and_plain_gap(self):
        client = FakeClient()
        source = FakeSocket(cap3_record() + cap3_record())
        with patch.object(vofa.socket, 'create_connection', return_value=source), \
                patch.object(vofa.select, 'select', side_effect=[([source], [], []), ([source], [], []), ([client], [], []), ([client], [], [])]):
            vofa.forward_live(client, 12345, None)

        lines = client.sent.decode('ascii').splitlines()
        self.assertEqual(len(lines), 513)
        self.assertEqual(lines[256], 'nan,nan')
        for line in lines:
            self.assert_two_numeric_columns(line)
        self.assertEqual(lines[0], '0,0')
        self.assertEqual(lines[1], '-1,1')
        self.assertEqual(lines[257], '0,0')


if __name__ == '__main__':
    unittest.main()
