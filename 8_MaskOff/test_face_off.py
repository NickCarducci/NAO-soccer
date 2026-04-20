#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
# -*- coding: utf-8 -*-
import imp
import os
import sys
import types
import unittest


class FakeALProxy(object):
    def __init__(self, *args, **kwargs):
        pass


fake_naoqi = types.ModuleType("naoqi")
fake_naoqi.ALProxy = FakeALProxy
sys.modules["naoqi"] = fake_naoqi

MODULE_PATH = os.path.join(os.path.dirname(__file__), "face_off.py")
face_off = imp.load_source("face_off_under_test", MODULE_PATH)


class FakeMemory(object):
    def __init__(self, data_map=None):
        self.data_map = data_map or {}
        self.inserted = {}

    def getData(self, key):
        value = self.data_map.get(key, [])
        if isinstance(value, Exception):
            raise value
        if callable(value):
            return value()
        return value

    def insertData(self, key, value):
        self.inserted[key] = value
        self.data_map[key] = value


class FakeFaceProxy(object):
    def __init__(self, learned=None, learn_results=None):
        self.learned = learned or []
        self.learn_results = list(learn_results or [])
        self.learn_calls = []

    def getLearnedFacesList(self):
        return list(self.learned)

    def learnFace(self, name):
        self.learn_calls.append(name)
        if self.learn_results:
            result = self.learn_results.pop(0)
        else:
            result = True
        if result and name not in self.learned:
            self.learned.append(name)
        return result


class FakeTTS(object):
    def __init__(self):
        self.spoken = []

    def say(self, text):
        self.spoken.append(text)


class FakeResponse(object):
    def __init__(self, payload):
        self.payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        return self.payload


class FaceOffTests(unittest.TestCase):
    def setUp(self):
        self.original_read_terminal_name_nonblocking = face_off.read_terminal_name_nonblocking
        self.original_bumper_pressed = face_off.bumper_pressed
        self.original_learn_face = face_off.learn_face
        self.original_clear_memory_key = face_off.clear_memory_key
        face_off.tts = FakeTTS()
        face_off.memory = FakeMemory()
        face_off.face_proxy = FakeFaceProxy()
        face_off.requests = None

    def tearDown(self):
        face_off.read_terminal_name_nonblocking = self.original_read_terminal_name_nonblocking
        face_off.bumper_pressed = self.original_bumper_pressed
        face_off.learn_face = self.original_learn_face
        face_off.clear_memory_key = self.original_clear_memory_key

    def test_face_detected_true_when_face_list_present(self):
        data = [0, [["shape_info"], [2, ["alex"]]]]
        self.assertTrue(face_off.face_detected(data))

    def test_get_recognized_faces_returns_names(self):
        one_face = [0, [["shape_info"], [2, ["alex"]]]]
        multi_face = [0, [["shape_info"], [3, ["alex", "mia"]]]]
        self.assertEqual(face_off.get_recognized_faces(one_face), ["alex"])
        self.assertEqual(face_off.get_recognized_faces(multi_face), ["alex", "mia"])

    def test_face_is_unrecognized_detects_marker(self):
        unknown_face = [0, [["shape_info"], [4]]]
        known_face = [0, [["shape_info"], [2, ["alex"]]]]
        self.assertTrue(face_off.face_is_unrecognized(unknown_face))
        self.assertFalse(face_off.face_is_unrecognized(known_face))

    def test_validate_candidate_name_accepts_any_non_empty_name(self):
        self.assertEqual(face_off.validate_candidate_name("Professor X", False), "professor x")

    def test_validate_candidate_name_rejects_duplicate_when_not_allowed(self):
        face_off.face_proxy = FakeFaceProxy(["alex"])
        self.assertIsNone(face_off.validate_candidate_name("alex", False))
        self.assertEqual(face_off.validate_candidate_name("alex", True), "alex")

    def test_ask_for_name_prefers_terminal_input_without_blocking(self):
        calls = []

        def fake_read_terminal(allow_known_names):
            calls.append("typed")
            return "typed name"

        face_off.read_terminal_name_nonblocking = fake_read_terminal
        face_off.memory = FakeMemory({"WordRecognized": ["spoken name", 0.9]})
        result = face_off.ask_for_name("Say a name.", False, 0.2)
        self.assertEqual(result, "typed name")
        self.assertEqual(calls, ["typed"])

    def test_ask_for_name_uses_spoken_name_when_no_terminal_input(self):
        face_off.read_terminal_name_nonblocking = lambda allow_known_names: None
        face_off.clear_memory_key = lambda key: None
        face_off.memory = FakeMemory({"WordRecognized": ["alex", 0.9]})
        result = face_off.ask_for_name("Say a name.", False, 0.2)
        self.assertEqual(result, "alex")

    def test_learn_face_retries_until_success(self):
        face_off.face_proxy = FakeFaceProxy(learn_results=[False, True])
        face_off.bumper_pressed = lambda: False
        result = face_off.learn_face("alex")
        self.assertTrue(result)
        self.assertEqual(face_off.face_proxy.learn_calls, ["alex", "alex"])

    def test_ensure_known_faces_only_teaches_missing_names(self):
        face_off.face_proxy = FakeFaceProxy(["first human"])
        taught = []

        def fake_learn_face(name):
            taught.append(name)
            return True

        face_off.learn_face = fake_learn_face
        self.assertTrue(face_off.ensure_known_faces())

        self.assertEqual(taught, ["second human"])

    def test_get_chatgpt_greeting_returns_fallback_without_requests(self):
        greeting = face_off.get_chatgpt_greeting("alex", None)
        self.assertEqual(greeting, "Hello, alex!")

    def test_get_chatgpt_greeting_parses_response(self):
        class FakeRequests(object):
            @staticmethod
            def post(*args, **kwargs):
                return FakeResponse({
                    "choices": [{"message": {"content": "Hello there, Alex."}}]
                })

        face_off.requests = FakeRequests()
        greeting = face_off.get_chatgpt_greeting("alex", "token")
        self.assertEqual(greeting, "Hello there, Alex.")


if __name__ == "__main__":
    unittest.main()
