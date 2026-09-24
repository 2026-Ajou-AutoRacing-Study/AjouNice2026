# 팀장 정적 장애물 회피 재현 시나리오

팀장님이 planning 정적 장애물 회피를 점검할 때 사용한
`static obstacle avoidance scenarios.zip`을 2026-09-24에 가져온 원본 재현 묶음이다.
대상 map은 `R_KR_PR_K-city_2025`이며, 5개 asset마다 `pos1--pos11`의 11개 배치를
포함해 총 55개다.

MORAI scenario 목록에서 asset별로 정렬되도록 파일명 앞에 다음 prefix만 추가했다.
JSON payload는 수정하지 않았다.

| Prefix | Asset | DataID |
|---|---|---:|
| `1_` | `Concrete_Barricade01` | `40100047` |
| `2_` | `OBJ_Hyundai_Sonata` | `40100013` |
| `3_` | `PE_Drum` | `40100027` |
| `4_` | `Scooter_01` | `40100062` |
| `5_` | `Woodbox` | `40100002` |

원본 ZIP SHA-256은
`45b0eafa0b34c7bd456265838456c93263f5f32111ec70584732cdd4b4bec0f6`이다.

이 파일들은 obstacle 하나만 남긴 minimal benchmark가 아니다. 팀장님 실험 조건을
그대로 재현하기 위해 원본의 pedestrian, traffic spawn point, shaded area와 waypoint도
보존한다. Detector/tracker/planning 문제를 분리할 때 임의로 scene 요소를 제거하지 말고,
먼저 동일 scene의 BEVFusion 성공·실패 run을 각각 기록한다.

로컬 MR 설치 위치는 다음과 같다.

```text
/home/luv/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/
```
