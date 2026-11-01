import copy
import importlib.util
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('receive',ROOT/'scripts/receive.py')
rx=importlib.util.module_from_spec(spec);spec.loader.exec_module(rx)


class ReceiverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines=(ROOT/'tests/et_wire.bin').read_text(encoding='ascii').splitlines()

    def parse(self, lines):
        d=rx.ETFrameDecoder()
        for line in lines:d.feed(line)
        d.finish()
        return d

    def test_valid_frame_and_gaps(self):
        d=self.parse(self.lines)
        self.assertEqual(len(d.completed),1);self.assertFalse(d.rejected)
        f=d.completed[0];self.assertEqual(f['expected_ready'],50)
        self.assertEqual(sum(p['ch0'] is None for p in f['points']),14)
        for p in f['points']:
            if p['valid']:self.assertEqual(p['ch0'],p['index']-32)

    def test_zero_is_data_not_gap(self):
        f=self.parse(self.lines).completed[0]
        self.assertTrue(f['points'][32]['valid'])
        self.assertEqual(f['points'][32]['ch0'],0)

    def test_single_hit_not_ready(self):
        p=self.parse(self.lines).completed[0]['points'][3]
        self.assertEqual(p['hits'],1);self.assertFalse(p['valid'])
        self.assertIsNone(p['ch0'])

    def test_zero_filled_gap_rejected(self):
        lines=self.lines.copy();lines[2]='wave:+0000,+0000'
        self.assertFalse(self.parse(lines).completed)

    def test_nan_in_qualified_bin_rejected(self):
        lines=self.lines.copy();lines[4]='wave:nan,nan'
        self.assertFalse(self.parse(lines).completed)

    def test_duplicate_index(self):
        lines=self.lines.copy();lines[3]=lines[1]
        self.assertFalse(self.parse(lines).completed)

    def test_missing_bin(self):
        lines=self.lines[:3]+self.lines[5:]
        self.assertFalse(self.parse(lines).completed)

    def test_trailing_partial(self):
        d=self.parse(self.lines[:-1])
        self.assertFalse(d.completed);self.assertEqual(len(d.rejected),1)

    def test_new_meta_does_not_join_frames(self):
        d=self.parse(self.lines[:20]+self.lines)
        self.assertEqual(len(d.completed),1);self.assertEqual(len(d.rejected),1)

    def test_corrupted_counts(self):
        lines=self.lines.copy();lines[3]=lines[3].replace('00160','00001')
        self.assertFalse(self.parse(lines).completed)

    def test_meta_occupancy_mismatch(self):
        lines=self.lines.copy();lines[0]=lines[0].replace('etv=0050','etv=0049')
        self.assertFalse(self.parse(lines).completed)

    def test_unlocked_mask_not_accepted(self):
        lines=self.lines.copy();lines[0]=lines[0].replace('etm=3','etm=1')
        self.assertFalse(self.parse(lines).completed)

    def test_unqualified_reference_not_accepted(self):
        lines=self.lines.copy();lines[0]=lines[0].replace('etq=1','etq=0')
        self.assertFalse(self.parse(lines).completed)

    def test_fft_format_identity(self):
        line=self.lines[0]
        proto=rx.decode(line)['protocol']
        self.assertIn(proto,(9,10))
        other=line.replace('proto=09','proto=10') if proto==9 else line.replace('proto=10','proto=09')
        with self.assertRaises(ValueError):rx.decode(other)

    def test_legacy_has_no_mask_claim(self):
        # The old wire format is readable as telemetry but is not a qualified
        # ET frame: there is no per-bin validity evidence to reconstruct.
        line=self.lines[0].replace('proto=09','proto=7').replace('proto=10','proto=8')
        self.assertFalse(self.parse([line]+self.lines[1:]).completed)

    def test_binary_or_bad_row(self):
        lines=self.lines.copy();lines[4]='wave:+9999,+0000'
        self.assertFalse(self.parse(lines).completed)


if __name__=='__main__':unittest.main(verbosity=2)
