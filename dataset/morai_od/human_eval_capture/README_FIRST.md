# Raw rosbag 수집 시작점

팀원은 아래 handoff를 처음부터 순서대로 따른다.

[`../docs/260923_HUMAN_DRIVING_ROSBAG_CAPTURE_HANDOFF.md`](../docs/260923_HUMAN_DRIVING_ROSBAG_CAPTURE_HANDOFF.md)

최초 준비 이후 실제 녹화 조작은 다음 네 명령뿐이다.

```bash
dataset/morai_od/human_eval_capture/capture.sh list
dataset/morai_od/human_eval_capture/capture.sh start H01 --driver <이름>
dataset/morai_od/human_eval_capture/capture.sh status
dataset/morai_od/human_eval_capture/capture.sh stop
```

BEVFusion, Docker, checkpoint, planning, localization을 실행하지 않는다.
