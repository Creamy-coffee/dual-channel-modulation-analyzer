import io
import unittest
from unittest.mock import patch
from urllib.error import URLError
import launch

class TestLaunch(unittest.TestCase):
    def test_reuses_existing_service(self):
        with patch.object(launch.urllib.request, 'urlopen', side_effect=[
            io.BytesIO(b'{"busy":true}'), io.BytesIO('载波图片发送'.encode())
        ]), patch.object(launch.webbrowser, 'open') as browser, patch.object(launch.runpy, 'run_path') as runner:
            self.assertEqual(launch.main(), 0)
            browser.assert_called_once_with(launch.URL)
            runner.assert_not_called()

    def test_starts_service_when_absent(self):
        with patch.object(launch.urllib.request, 'urlopen', side_effect=URLError('offline')), patch.object(launch.runpy, 'run_path') as runner, patch.object(launch.sys, 'argv', []):
            self.assertEqual(launch.main(), 0)
            runner.assert_called_once()

    def test_other_service_is_preserved(self):
        with patch.object(launch.urllib.request, 'urlopen', side_effect=[
            io.BytesIO(b'{}'), io.BytesIO(b'other service')
        ]), patch.object(launch.runpy, 'run_path') as runner:
            self.assertEqual(launch.main(), 1)
            runner.assert_not_called()

if __name__ == '__main__':
    unittest.main()
