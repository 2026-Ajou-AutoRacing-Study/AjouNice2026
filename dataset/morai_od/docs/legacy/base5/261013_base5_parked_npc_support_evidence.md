# Base5 정지 NPC 도로 지지 보완과 검증

문서 상태: EVIDENCE. 기준일: 2026-10-13.

Sunny13에서 긴 정지 NPC의 도로 밖 배치를 보완하고 실제 센서 캡처·정지·충돌·복구를 검증했다.
Sunny13 5K 본수집 준비는 완료했지만 실행하지 않았다. 기존 Sunny13 5K와 모델·GT 정책은 보존했다.
다른 다섯 환경은 같은 보완의 오프라인 배치만 생성했으며 실제 날씨별 검증은 별도로 수행한다.

## 재연결과 annotation 저장

재연결 후 Ego/Object/CollisionData와 EventCmd는 수신됐지만 첫 요청은 RGB 4장·LiDAR 원본 1개만
저장됐다. 카메라 Instance와 LiDAR Instance annotation이 없어 `Synchronous file completeness timeout`으로
중단했고 최종 Park는 확인했다. 사용자에게 GT 저장을 다시 켜도록 요청한 뒤 새 output에서 재검사했다.

첫 시도의 7개 파일은 `sensor_annotation_missing_archive_v150`에 복사하고 원본도 보존했다.
이 묶음은 학습·승인 데이터가 아니며 나중에 다른 시점의 annotation을 붙이지 않는다.
Capture Mode와 Instance/GT 저장은 다른 설정이다. MR 재시작 뒤 통신이 된다고 GT 저장도 켜졌다고 가정하지 않는다.

## 긴 NPC 배치 정책

`long-parked-npc-road-support-v150`는 정적 회피와 음영의 정지 물리 NPC 중 길이 8m 이상을 선별한다.
MGeo에서 차체 네 모서리의 도로 지지를 살펴 두 모서리 이상이 도로 바깥 추정이거나 가장 가까운
도로에서 2.5m보다 멀면 오프라인에서 새 위치를 샘플링한다. 한 모서리의 작은 지도 불확실성은 허용한다.

기존 자유 yaw가 해당 긴 차체를 도로 밖으로 걸치게 한 경우에만 해당 actor의 도로 방향 fallback을
명시적으로 기록한다. 모든 버스·차량을 같은 방향으로 고정하지 않는다. 고주로·로터리의 경로 기반 차량,
일반 범용 차량, passive OBJ 자산, 사람·동물·복합체 및 실제 이동 경로는 이 보완에서 제외한다.
일반 장애물의 자유 yaw도 유지한다.

오프라인 후보를 두 번 수정했다. 첫 후보에서는 shadow/avoidance의 yaw 계산에도 제원 전달이 필요함을
확인해 두 호출 경로를 함께 보완했다. 두 번째 후보의 광범위한 지도 조건은 기존 로터리 snapshot을
만들 수 없게 했으므로 기각하고, 의도적으로 정지 yaw를 다양화하는 위 두 구역으로 범위를 제한했다.
불완전한 `candidate_v150`, `candidate_v150b`는 진단 이력이며 수집 입력이 아니다.

최종 `candidate_v150c/sunny13`에서 5,000개 slot·primary·기존 actor 목록·거리/방향·split/group은 유지됐다.
24장만 재배치했고 나머지 4,976장의 payload SHA는 동일하다. 24장 중 음영 12장과 정적 회피 12장이며
15개 actor의 yaw fallback을 기록했다. 기존 도로 방향을 유지하며 위치만 다시 잡은 경우도 있다.
정적 제원, 크기 budget, 19종 장애물 보강과 Native GT는 변경하지 않았다.

## 실제 캡처 결과

| 검사 | 결과 | 근거 |
|---|---:|---|
| 도로 방향 긴 버스 4장 | 4/4 승인, 약 19초 | XY pivot 최대 편차 1.1~1.3cm, LiDAR 92~323점, 접촉 없음 |
| 보완된 slot 전체 | 24/24 승인 | 최종 계획 payload 그대로 검사, 미션·자산 제외 없이 검증 |
| 정적 회귀 64장 | 최초 63/64 승인 | 보완 24장과 기존 등록 순서의 대조 40장 |
| 실제 부족분 보충 | 1회 새 후보로 64/64 | 부모 63장 유지, 실패 raw 유지, Native QA 완화 없음 |
| 실제 이동 회귀 | 8/8 승인, 약 266초 | 교차로·음영 각 4장, 기존 높이·이동·endpoint 검사 유지 |
| 중간 일시정지 후 resume | 기존 16장과 원본 유지 | Native intensity 16→68개는 이후 촬영·QA이며 기존 파일 SHA는 불변 |
| 완료 후 resume | 추가 촬영 0장 | Native intensity 69→69개, 부모와 보충 ledger SHA 유지 |

보완 slot의 정지 NPC 관측 34개에서 계획 pivot 대비 XY 편차는 중앙값 4.4mm, p90 11.8mm,
p95 14.0mm, 최대 20.5mm였다. 이는 관측된 정지 위치 편차이지 모든 tyre/mesh 접지 인증이 아니다.
버스 4장의 local ground plane은 확인하지 못했고 기존 정적 advisory 경고로 보존했다.
심각한 높이·실제 이동·충돌·통신/Park 조건을 완화하지 않았다.

부족분은 낮은 `Lane_Separation_Bar`를 30~60m 후방에 둔 장면의 가시성 실패였다. 보충에서는 동일
자산을 10~30m 전방 혼합 장면으로 새로 배치했고 actor 1→4, 방향·거리 변경을 mapping에 기록했다.
64장 승인은 원래 exact 거리·개수 quota를 모두 채웠다는 뜻이 아니다. 동일 실패 payload는 재촬영하지 않았다.

## 다음 환경과 비용

다른 다섯 환경은 `candidate_v150c/future_suite`에 각 5K로 생성했다.

| 환경 | 긴 NPC 때문에 재배치한 장면 |
|---|---:|
| Sunny11 | 26 |
| Sunny15 | 17 |
| Foggy11 | 22 |
| Foggy13 | 25 |
| Foggy15 | 24 |

각 환경의 logical slot·자산·동작·split 보존 감사는 오류 0개다. 기존 완료 Sunny13를 유지해 총 30K 계획을
유지한다. 별도 새 Sunny13 preview를 자동 추가하거나 이미 확보한 5K를 삭제하지 않는다.

이번 64+8장 회귀를 실제 정적/교차로 이동/음영 이동 비율로 가중하면 5K 약 8.14시간·31.25GiB다.
실제 보충 1회에는 추가 11.2초가 들었고 해당 비용·저장량을 release에 별도로 기록했다.
v149의 11.37시간과는 표본 구성도 달라 순수 속도 개선율로 해석하지 않는다. 운영 예산은
8~12시간·45GiB로 잡고 본수집 초반 rate로 갱신한다. 현재 여유 약 110GiB로 25K 전체를 동시에 보관하기는 부족하다.

## 동결 release와 보존

실험 root는 `~/morai_cap/base5_obstacle_improvement_261013`이다.
현재 `release_v150_sunny13/release.json`이 검증된 Sunny13 진입점이며 `bulk_ready=true`, `bulk_started=false`다.
생성 폴더 이름은 `candidate_v150c`지만 이 release가 해시로 검증·동결한 입력이다. 임의로 경로를 바꾸지 않는다.
구 v149 release를 수정하거나 현재 코드로 강제 resume하지 않는다.

최종 단위 검사 697개, scripts/tests의 `compileall`, `git diff --check`가 통과했다.
동결 release의 CPU prepare-only도 통과했고 새 5K output은 `finished=0`, `accepted_candidate=0`이다.
기존 Sunny13 selected ledger의 SHA256은
`026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43`로 유지됐다.
원본·실패·보충·진단 package는 git 밖에서 보존하며 planning validator와 모델 배포 설정은 변경하지 않았다.

명령과 재연결 조건은 [현재 수집 런북](261013_base5_obstacle_support_runbook.md)을 따른다.
