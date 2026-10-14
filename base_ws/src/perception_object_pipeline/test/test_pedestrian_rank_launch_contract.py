from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

PACKAGE=Path(__file__).resolve().parents[1]
TRACKER=PACKAGE.parent/'ekf-multi-object-tracking'


class PedestrianRankLaunchContract(unittest.TestCase):
    def test_production_argument_defaults_on_and_forwards(self):
        root=ET.parse(PACKAGE/'launch/object_pipeline.launch').getroot()
        arg=root.find("arg[@name='pedestrian_gaussian_association_ranking']")
        self.assertEqual(arg.get('default'),'true')
        include=next(i for i in root.findall('include') if 'ekf_multi' in i.get('file'))
        self.assertEqual(include.find("arg[@name='pedestrian_gaussian_association_ranking']").get('value'),
                         '$(arg pedestrian_gaussian_association_ranking)')

    def test_wrapper_has_explicit_rollback(self):
        root=ET.parse(PACKAGE/'launch/tracking_prediction.launch').getroot()
        self.assertEqual(root.find("arg[@name='tracker_pedestrian_gaussian_association_ranking']").get('default'),'true')
        include=next(i for i in root.findall('include') if 'object_pipeline.launch' in i.get('file'))
        self.assertEqual(include.find("arg[@name='pedestrian_gaussian_association_ranking']").get('value'),
                         '$(arg tracker_pedestrian_gaussian_association_ranking)')

    def test_generic_tracker_stays_explicitly_opt_in(self):
        root=ET.parse(TRACKER/'launch/ekf_multi_class_object_tracking.launch').getroot()
        self.assertEqual(root.find("arg[@name='pedestrian_gaussian_association_ranking']").get('default'),'false')
        param=root.find("param[@name='configure/pedestrian_gaussian_association_ranking']")
        self.assertEqual(param.get('value'),'$(arg pedestrian_gaussian_association_ranking)')


if __name__=='__main__':unittest.main()
