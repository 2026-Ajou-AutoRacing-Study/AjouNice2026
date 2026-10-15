import unittest
from profile_pipeline_cpu_tracker_visualization import function_source


class TrackerVisualizationExtractionTest(unittest.TestCase):
    def test_extracts_nested_body_without_next_function(self):
        text='void Node::method() {\n    if (true) {\n        work();\n    }\n}\nvoid Node::next() {\n}\n'
        result=function_source(text,'void Node::method')
        self.assertTrue(result.endswith('\n}'))
        self.assertIn('work();',result)
        self.assertNotIn('next',result)

    def test_source_change_missing_or_duplicate_fails_closed(self):
        for text in ('void Other::method() {\n}\n',
                     'void Node::method() {\n}\nvoid Node::method(int a) {\n}\n'):
            with self.assertRaises(ValueError):function_source(text,'void Node::method')


if __name__=='__main__':unittest.main()
