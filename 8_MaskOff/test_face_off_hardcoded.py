import unittest
import sys
import types
from mock import MagicMock, patch

# Patch sys.modules to mock naoqi import
sys.modules['naoqi'] = types.ModuleType('naoqi')
sys.modules['naoqi'].ALProxy = MagicMock()

import face_off_hardcoded

class TestFaceOffHardcoded(unittest.TestCase):
    def setUp(self):
        # Patch all proxies and robot methods
        self.tts = MagicMock()
        self.memory = MagicMock()
        self.camera = MagicMock()
        self.face_proxy = MagicMock()
        self.sound_proxy = MagicMock()
        self.motion = MagicMock()
        face_off_hardcoded.tts = self.tts
        face_off_hardcoded.memory = self.memory
        face_off_hardcoded.camera = self.camera
        face_off_hardcoded.face_proxy = self.face_proxy
        face_off_hardcoded.sound_proxy = self.sound_proxy
        face_off_hardcoded.motion = self.motion

    def test_bumper_pressed_true(self):
        self.memory.getData.side_effect = lambda key: 1.0 if key in ["RightBumperPressed", "LeftBumperPressed"] else 0.0
        self.assertTrue(face_off_hardcoded.bumper_pressed())

    def test_bumper_pressed_false(self):
        self.memory.getData.side_effect = lambda key: 0.0
        self.assertFalse(face_off_hardcoded.bumper_pressed())

    def test_get_recognized_faces(self):
        # Simulate recognized face data
        face_data = [None, [[], [2, ["Bob"]]]]
        result = face_off_hardcoded.get_recognized_faces(face_data)
        self.assertEqual(result, ["Bob"])

    def test_turn_toward_sound(self):
        self.memory.getData.return_value = [None, [0.5, 0.1]]
        face_off_hardcoded.turn_toward_sound()
        self.motion.setAngles.assert_called_with(["HeadYaw", "HeadPitch"], [0.5, -0.1], 0.3)

    def test_main_learning_and_greeting(self):
        # Patch face_proxy.learnFace to succeed
        self.face_proxy.learnFace.side_effect = [True, True]
        # Patch get_recognized_faces to return Bob on first loop, then []
        with patch.object(face_off_hardcoded, 'get_recognized_faces', side_effect=[["Bob"], []]):
            # Patch bumper to break after greeting
            with patch.object(face_off_hardcoded, 'bumper_pressed', side_effect=[False, True]):
                with patch('time.sleep', return_value=None):
                    face_off_hardcoded.main()
        # Check learning called for Bob and Larry
        self.assertEqual(self.face_proxy.learnFace.call_count, 2)
        # Check greeting was spoken
        self.tts.say.assert_any_call("Hello, Bob!")

if __name__ == "__main__":
    unittest.main()
