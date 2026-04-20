import unittest
from seek import parse_object_label, OBJECT_PREFIX_MESSAGES, SIDES

class TestSeekParsing(unittest.TestCase):
    def test_page_shape_prefix(self):
        prefix, side, suffix, message = parse_object_label("page-shape-front")
        self.assertEqual(prefix, "page-shape-")
        self.assertEqual(side, "front")
        self.assertEqual(message, OBJECT_PREFIX_MESSAGES["page-shape-"])

    def test_page_prefix(self):
        prefix, side, suffix, message = parse_object_label("page-back")
        self.assertEqual(prefix, "page-")
        self.assertEqual(side, "back")
        self.assertEqual(message, OBJECT_PREFIX_MESSAGES["page-"])

    def test_no_side(self):
        prefix, side, suffix, message = parse_object_label("book-1")
        self.assertEqual(prefix, "book-")
        self.assertIsNone(side)
        self.assertEqual(message, OBJECT_PREFIX_MESSAGES["book-"])

    def test_unknown_prefix(self):
        prefix, side, suffix, message = parse_object_label("unknown-object")
        self.assertIsNone(prefix)
        self.assertIsNone(side)
        self.assertEqual(message, "I found an object.")

    def test_all_sides(self):
        for side in SIDES:
            label = "cube-" + side
            prefix, found_side, suffix, message = parse_object_label(label)
            self.assertEqual(prefix, "cube-")
            self.assertEqual(found_side, side)
            self.assertEqual(message, OBJECT_PREFIX_MESSAGES["cube-"])

if __name__ == "__main__":
    unittest.main()
