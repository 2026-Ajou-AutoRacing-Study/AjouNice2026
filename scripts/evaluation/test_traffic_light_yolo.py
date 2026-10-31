"""Pure YOLO dataset/matching/reproducibility gates; never import YOLO/Torch."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from traffic_light_yolo_dataset import load_dataset,normalized_label,match_detections,detection_summary
from train_traffic_light_yolo import training_arguments
from evaluate_traffic_light_yolo import timing_summary,confidence_analysis,confidence_distribution
from select_traffic_light_yolo_checkpoint import selection_key
from restore_traffic_light_yolo_checkpoint_parts import restore
from diagnose_traffic_light_yolo_predictions import analyze
from ablate_traffic_light_yolo_hsv_margin import eligible,summarize


class DetectionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.path=self.root/"labels.json";self.rows=[]
        for index,split in enumerate(("train","train","val","test")):
            p=self.root/(str(index)+".jpg");p.write_bytes(("fixture-"+str(index)).encode())
            self.rows.append({"image_path":p.name,"image_sha256":hashlib.sha256(p.read_bytes()).hexdigest(),
                "width":1280,"height":720,"split":split,"group_id":"pose"+str(index),"environment":"Sunny13",
                "annotation_complete":True,"boxes":[{"xyxy":[100,50,160,70],"class_id":0,
                    "head_role":"near","box_qc_accepted":True},{"xyxy":[600,200,630,208],"class_id":0,
                    "head_role":"far","box_qc_accepted":True}]})
    def load(self):
        self.path.write_text(json.dumps({"records":self.rows}))
        return load_dataset(self.path,size_reader=lambda _:(1280,720))
    def test_multiple_boxes_original_hash_and_group_split(self):
        rows,metadata=self.load();self.assertEqual(len(rows[0]["boxes"]),2);self.assertEqual(metadata["box_count"],8)
        self.assertEqual(metadata["split_environment_image_counts"]["train"],{"Sunny13":2})
        self.assertEqual(metadata["split_box_counts"]["val"],2)
    def test_exported_normalized_labels_cannot_drift_from_manifest(self):
        for index,row in enumerate(self.rows):
            source=self.root/row["image_path"]
            image=self.root/"images"/row["split"]/(str(index)+".jpg")
            image.parent.mkdir(parents=True,exist_ok=True);image.write_bytes(source.read_bytes())
            label=self.root/"labels"/row["split"]/(str(index)+".txt")
            label.parent.mkdir(parents=True,exist_ok=True)
            label.write_text("".join(normalized_label(box,1280,720)+"\n" for box in row["boxes"]))
            row.update(image_path=str(image.relative_to(self.root)),label_sha256=hashlib.sha256(label.read_bytes()).hexdigest())
        self.load()
        label=self.root/"labels"/"train"/"0.txt";label.write_text("0 0.5 0.5 0.1 0.1\n")
        with self.assertRaises(ValueError):self.load()
    def test_partial_annotations_and_unreviewed_boxes_rejected(self):
        self.rows[0]["annotation_complete"]=False
        with self.assertRaises(ValueError):self.load()
        self.rows[0]["annotation_complete"]=True;self.rows[0]["boxes"][0]["box_qc_accepted"]=False
        with self.assertRaises(ValueError):self.load()
    def test_same_image_and_pose_never_cross_splits(self):
        self.rows[3]["group_id"]=self.rows[0]["group_id"]
        with self.assertRaises(ValueError):self.load()
        self.rows[3]["group_id"]="new";(self.root/"3.jpg").write_bytes((self.root/"0.jpg").read_bytes())
        self.rows[3]["image_sha256"]=self.rows[0]["image_sha256"]
        with self.assertRaises(ValueError):self.load()
    def test_bad_sha_and_outside_or_zero_box_rejected(self):
        for box in ([0,0,0,10],[-1,0,20,10],[0,0,1281,10],[0,0,float("nan"),10]):
            self.rows[0]["boxes"][0]["xyxy"]=box
            with self.subTest(box=box),self.assertRaises(ValueError):self.load()
    def test_empty_box_list_only_with_complete_frame(self):
        self.rows[0]["boxes"]=[];rows,_=self.load();self.assertEqual(rows[0]["boxes"],[])
    def test_duplicate_detection_not_double_recall_and_near_far_bins(self):
        truth=self.rows[0]["boxes"];pred=[{"xyxy":truth[0]["xyxy"],"confidence":.9},
                                             {"xyxy":truth[0]["xyxy"],"confidence":.8}]
        matched=match_detections(truth,pred)
        self.assertEqual(len(matched["matches"]),1);self.assertEqual(len(matched["false_positive_indices"]),1)
        report=detection_summary([{"truth_boxes":truth,"matching":matched}])
        self.assertEqual(report["recall_at_fixed_conf"],.5)
        self.assertEqual(report["GT_box_bins"]["role:near"]["recall"],1)
        self.assertEqual(report["GT_box_bins"]["role:far"]["recall"],0)
        self.assertEqual(report["GT_box_bins"]["height:le8"]["gt"],1)
    def test_normalized_box_and_explicit_reproducible_training(self):
        label=normalized_label(self.rows[0]["boxes"][0],1280,720).split()
        self.assertEqual(label[0],"0");self.assertAlmostEqual(float(label[1]),130/1280)
        config=training_arguments("data.yaml","new")
        self.assertFalse(config["amp"]);self.assertFalse(config["compile"]);self.assertFalse(config["cache"])
        self.assertEqual(config["split"],"val");self.assertEqual(config["fliplr"],0)
        self.assertEqual(config["imgsz"],960)
        self.assertTrue(Path(config["project"]).is_absolute())
    def test_fixed_epoch_snapshots_and_same_default_nms_head(self):
        config=training_arguments("data.yaml","new",epochs=100,patience=0,save_period=10)
        self.assertEqual(config["epochs"],100);self.assertEqual(config["patience"],0)
        self.assertEqual(config["save_period"],10);self.assertIsNone(config["nms"])
        with self.assertRaises(ValueError):training_arguments("data.yaml","new",save_period=0)
    def test_timing_percentile_is_not_unsynchronized_engine_estimate(self):
        summary=timing_summary([1,2,3,4,100]);self.assertEqual(summary["p95_ms"],100)
    def test_score_diagnostics_separate_tp_fp_and_rematch_thresholds(self):
        truth=self.rows[0]["boxes"]
        predictions=[{"xyxy":truth[0]["xyxy"],"confidence":.02},
                     {"xyxy":[0,0,10,10],"confidence":.005}]
        row={"truth_boxes":truth,"predictions":predictions,"matching":match_detections(truth,predictions)}
        report=confidence_analysis([row],.001)
        self.assertEqual(report["matched_TP_scores"]["p50"],.02)
        self.assertEqual(report["unmatched_FP_scores"]["max"],.005)
        by_conf={r["conf"]:r for r in report["threshold_sweep"]}
        self.assertEqual(by_conf[.01]["tp"],1);self.assertEqual(by_conf[.01]["fp"],0)
        self.assertEqual(by_conf[.25]["fn"],2)
        self.assertIsNone(confidence_distribution([])["p50"])
    def test_non_mission_correct_detection_is_not_detector_false_positive(self):
        truth=copy.deepcopy(self.rows[0]["boxes"])
        truth[0]["native_id"]="C1256W000016";truth[1]["native_id"]="C1256W000047"
        predictions=[{"xyxy":box["xyxy"],"confidence":.9} for box in truth]
        report=detection_summary([{"truth_boxes":truth,"matching":match_detections(truth,predictions)}])
        self.assertEqual(report["fp"],0);self.assertEqual(report["GT_box_bins"]["mission:allowed5"]["recall"],1)
        self.assertEqual(report["GT_box_bins"]["native:C1256W000016"]["matched"],1)
        self.assertEqual(report["matched_non_mission_head_count_not_detector_FP"],1)
        self.assertFalse(report["route_association_evaluated"])
    def test_operating_point_qualifies_before_map_tie_break(self):
        def candidate(tp,fp,fn,recall,map95):
            return {"operating_point":{"tp":tp,"fp":fp,"fn":fn,"GT_box_bins":{"mission:allowed5":{"recall":recall}}},
                    "official_mAP":{"mAP50_95":map95}}
        early=candidate(0,0,90,0.,.79);late=candidate(90,0,0,1.,.76)
        self.assertGreater(selection_key(late),selection_key(early))
        false_positive=candidate(90,3,0,1.,.9)
        self.assertGreater(selection_key(late),selection_key(false_positive))
        higher_map=candidate(90,0,0,1.,.8)
        self.assertGreater(selection_key(higher_map),selection_key(late))
    def test_lossless_checkpoint_parts_exact_hash_and_corruption_gate(self):
        raw=b"trusted-synthetic-checkpoint-fixture"*8;compressed=gzip.compress(raw,mtime=0)
        parts=self.root/"parts";parts.mkdir();items=[]
        for index,payload in enumerate((compressed[:15],compressed[15:])):
            name=str(index)+".gzpart";(parts/name).write_bytes(payload)
            items.append({"file":name,"bytes":len(payload),"sha256":hashlib.sha256(payload).hexdigest()})
        sha=hashlib.sha256(raw).hexdigest()
        (parts/"manifest.json").write_text(json.dumps({"source_sha256":sha,"source_bytes":len(raw),
            "gzip_bytes":len(compressed),"gzip_sha256":hashlib.sha256(compressed).hexdigest(),
            "parts":items,"lossless":True}))
        output=self.root/"restored.pt";restore(parts,output,sha);self.assertEqual(output.read_bytes(),raw)
        with self.assertRaises(FileExistsError):restore(parts,output,sha)
        (parts/"0.gzpart").write_bytes(b"corrupt")
        with self.assertRaises(ValueError):restore(parts,self.root/"bad.pt",sha)
        self.assertFalse((self.root/"bad.pt").exists())
    def test_forensic_loose_geometry_does_not_replace_strict_score(self):
        truth=[{"xyxy":[100,100,120,110],"head_role":"near","native_id":"C1256W000052"}]
        predictions=[{"xyxy":[100,104,120,114],"confidence":.95}]
        row={"truth_boxes":truth,"predictions":predictions,"matching":match_detections(truth,predictions),
             "group_id":"D2","environment":"Sunny13","phase_key":"UNKNOWN","image_path":"raw.jpg","image_sha256":"a"*64}
        report=analyze([row]);self.assertEqual(report["strict_IoU_0_5"]["fn"],1)
        self.assertEqual(report["geometry_IoU_0_25_DIAGNOSTIC_NOT_PIPELINE_IDENTITY"]["fn"],0)
        self.assertEqual(report["FP_geometric_categories"]["localization_overlap_0_25_to_0_5"],1)
        self.assertTrue(report["missed_GT"][0]["node_crop_contains_entire_GT"])
    def test_hsv_ablation_identity_native_gate_and_unknown_not_all_off(self):
        row={"map_association":{"target_identity_correct":True,"reference":{"known":True}}}
        self.assertTrue(eligible(row));row["map_association"]["reference"]["known"]=False
        self.assertFalse(eligible(row))
        metrics=summarize([{"truth_bits":[1,0,0,0],"p":None}],"p")
        self.assertEqual(metrics["unknown"],1);self.assertEqual(metrics["exact_correct"],0)


if __name__=="__main__":unittest.main()
