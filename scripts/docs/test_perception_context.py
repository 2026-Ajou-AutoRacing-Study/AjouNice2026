import unittest
from pathlib import Path

from check_perception_context import anchors, references, slug, ROOT, audit


class ContextTests(unittest.TestCase):
    def test_unicode_heading_and_punctuation(self):
        self.assertEqual(slug('현재 기준: Base5/3D'), '현재-기준-base53d')

    def test_explicit_and_duplicate_anchors(self):
        result=anchors('<a id="runtime"></a>\n## 같은 제목\n## 같은 제목\n')
        self.assertEqual(result, {'runtime','같은-제목','같은-제목-1'})

    def test_fenced_code_does_not_create_headings_or_anchors(self):
        result=anchors('## 실제 절\n```bash\n# 주석\n<a id="fake"></a>\n```\n')
        self.assertEqual(result, {'실제-절'})

    def test_relative_fragment_and_encoded_path(self):
        path=Path('/repo/docs/perception/README.md')
        result=list(references(path,'[link](../bevfusion/My%20Doc.md#runtime)'))
        self.assertEqual(result, [(Path('/repo/docs/bevfusion/My Doc.md'),'runtime')])

    def test_external_links_do_not_request_network(self):
        result=list(references(Path('/repo/README.md'),'[remote](https://example.com/x#y)'))
        self.assertEqual(result, [])

    def test_fragment_only_refers_to_same_file(self):
        path=Path('/repo/CURRENT.md')
        self.assertEqual(list(references(path,'[runtime](#runtime)')),[(path,'runtime')])

    def test_fenced_examples_are_not_mandatory_dependencies(self):
        text='```md\n[old](missing.md)\n```\n[current](CURRENT.md)'
        result=list(references(Path('/repo/README.md'),text))
        self.assertEqual(result, [(Path('/repo/CURRENT.md'),'')])

    def test_actual_context_graph_archives_and_budgets(self):
        report=audit(ROOT)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['preserved_originals'],13)
        self.assertLess(report['same_large_agent_lines_after'],report['archived_large_agent_lines_before']/4)
        self.assertGreater(report['checked_incoming_links'],0)


if __name__=='__main__':unittest.main()
