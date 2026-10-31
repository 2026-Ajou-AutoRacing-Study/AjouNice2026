"""Pure proposal tests: no Torch, CNN, GPU, ROS, files, or simulator."""
import copy
import unittest

from build_traffic_light_yolo_cnn_crops import node_crop,propose_records
from prepare_traffic_light_yolo_cnn_remote import remap_rows
from curate_traffic_light_yolo_cnn_proposals import approve


class ProposalTests(unittest.TestCase):
    def setUp(self):
        self.row={"image_path":"raw.jpg","image_sha256":"a"*64,"crop_xyxy":[70,70,140,140],
                  "split":"train","group_id":"pose1","environment":"Sunny13","native_gt_verified":True,
                  "manual_qc_accepted":True,"visual_labels_known":True,
                  "labels":{"red":1,"yellow":0,"green":0,"left_arrow":0},
                  "provenance":{"native_observed_id":"C1256W000016"}}
        self.housing={"split":"train","group_id":"pose1","width":1280,"height":720,
                      "boxes":[{"xyxy":[100,100,120,110],"native_id":"C1256W000016"}]}
        self.pred={"xyxy":[99.8,99.2,121.9,111.8],"confidence":.9,"class_id":0}
    def build(self,rows=None):
        return propose_records(rows or [self.row],{"a"*64:self.housing},{"a"*64:[self.pred]},
                               {"weights_sha256":"b"*64,"conf":.25})
    def test_node_integer_truncation_and_boundary_clipping(self):
        self.assertEqual(node_crop(self.pred["xyxy"],1280,720),[87,87,133,123])
        self.assertEqual(node_crop([.9,1.9,1279.9,719.9],1280,720),[0,0,1280,720])
    def test_valid_proposal_never_inherits_manual_approval_or_changes_original(self):
        original=copy.deepcopy(self.row);proposals,skips=self.build();self.assertEqual(skips,[])
        self.assertEqual(self.row,original);self.assertEqual(len(proposals),1)
        new=proposals[0];self.assertFalse(new["manual_qc_accepted"]);self.assertFalse(new["visual_labels_known"])
        self.assertTrue(new["native_gt_verified"]);self.assertEqual(new["labels"],self.row["labels"])
        self.assertEqual(new["split"],"train");self.assertEqual(new["crop_xyxy"],[87,87,133,123])
        self.assertTrue(new["provenance"]["yolo_crop_proposal"]["original_approved_crop_and_QC"]["manual_qc_accepted"])
    def test_test_and_manual_background_are_never_augmented(self):
        self.row["split"]="test";self.assertEqual(self.build()[1][0]["reason"],"test_not_augmented")
        self.row["split"]="train";self.row["native_gt_verified"]=False
        self.assertEqual(self.build()[1][0]["reason"],"not_native_positive")
    def test_crop_must_contain_full_housing_not_only_IoU_pass(self):
        self.housing["boxes"][0]["xyxy"]=[100,100,160,110]
        self.pred["xyxy"]=[100,100,141,110]
        self.assertEqual(self.build()[1][0]["reason"],"crop_does_not_contain_entire_QC_housing")
    def test_overlap_with_another_housing_is_held(self):
        self.housing["boxes"].append({"xyxy":[130,100,150,110],"native_id":"C1256W000018"})
        self.assertEqual(self.build()[1][0]["reason"],"crop_overlaps_other_QC_housing")
    def test_native_join_not_requested_id_or_nearest_box(self):
        self.row["provenance"]={"requested_native_id":"C1256W000016"}
        self.assertEqual(self.build()[1][0]["reason"],"missing_or_conflicting_observed_native_id")
        self.row["provenance"]={"native_observed_id":"C1256W000018"}
        self.assertEqual(self.build()[1][0]["reason"],"native_head_missing_or_ambiguous")
    def test_same_integer_crop_is_not_duplicated(self):
        self.row["crop_xyxy"]=[87,87,133,123]
        self.assertEqual(self.build()[1][0]["reason"],"duplicate_existing_or_proposed_integer_crop")
    def test_split_or_group_mismatch_fails_instead_of_reassigning(self):
        self.housing["split"]="val"
        with self.assertRaises(ValueError):self.build()
    def test_low_IoU_missing_detection_does_not_borrow_labels(self):
        self.pred["xyxy"]=[300,300,330,310]
        self.assertEqual(self.build()[1][0]["reason"],"target_not_matched_at_IoU_0_5")
    def test_native_all_off_or_unapproved_original_cannot_be_proposed(self):
        self.row["labels"]={key:0 for key in self.row["labels"]}
        self.assertEqual(self.build()[1][0]["reason"],"native_all_off_not_eligible")
        self.row["labels"]["red"]=1;self.row["manual_qc_accepted"]=False
        with self.assertRaises(ValueError):self.build()
    def test_remote_remap_reuses_SHA_and_preserves_labels_split_and_false_native(self):
        old=copy.deepcopy(self.row);old["native_gt_verified"]=False
        reference={"image_sha256":old["image_sha256"],"image_path":"images/train/000.jpg","split":"train","group_id":"pose1"}
        mapped,missing=remap_rows([old],[reference],"yolo_existing","extra")
        self.assertEqual(missing,{})
        self.assertEqual(mapped[0]["image_path"],"yolo_existing/images/train/000.jpg")
        self.assertFalse(mapped[0]["native_gt_verified"]);self.assertEqual(mapped[0]["labels"],old["labels"])
        self.assertEqual(old["image_path"],"raw.jpg")
        reference["split"]="val"
        with self.assertRaises(ValueError):remap_rows([old],[reference],"yolo_existing","extra")
    def test_explicit_root_review_required_and_original_prefix_immutable(self):
        with self.assertRaises(ValueError):approve({"records":[self.row]},{"records":[self.row]}, {}, {})
        modified=copy.deepcopy(self.row);modified["labels"]["red"]=0
        with self.assertRaises(ValueError):approve({"records":[self.row]},{"records":[modified]}, {}, {},root_visual_review_confirmed=True)


if __name__=="__main__":unittest.main()
