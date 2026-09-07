MORAI semantic split v1

Dataset: /home/jeon/ajouNice/AjouNice2026/data/morai_4class_v1

Rules:
- source images/masks are never modified, copied, renamed, or moved
- split files are manifests only
- CAMERA_1~4 with the same full sample key stay together
- full sample key is used because fXX tokens can repeat
- deterministic contiguous 70/15/15 split

Total: 476 groups / 1904 images
Train: 333 groups / 1332 images
Validation: 71 groups / 284 images
Test: 72 groups / 288 images
