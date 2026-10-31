# 도구 진입점

현재 도구는 [scripts 안내](../scripts/README.md), [dataset 안내](../dataset/morai_od/docs/README.md),
[BEVFusion 안내](../docs/bevfusion/README.md)에서 목적별로 선택한다.

`segmentation/`은 과거 PointPainting의 LR-ASPP 4-class 준비·학습 호환 경로다.
일부 함수가 기존 PointPainting 도구에서 import되므로 경로를 유지한다. 현재 Base5 필수 단계가 아니다.
과거 segmentation mask/split 도구는 `legacy/morai_dataset/`에 원문 보관했다.
