# Base5 사거리 수동 접지 경로 — PLAN / 준비 완료

2026-10-08. 이동 외관이 필요한 객체의 수집 비용을 줄이기 위한 수동 경로 보정 단계다.
현재는 보정 전 시나리오만 생성·설치했다. 수동 보정 결과, lookup, live 합격 또는 시간 개선 실측은 아직 없다.
기존 v115 정적 QA와 동적 엄격 검사를 유지한다. 본5K·학습·production 변경은 하지 않는다.

## 이번 수집 방향

- 실제 이동 외관 캡처는 우선 동적 장애물 미션의 신호등 사거리 두 횡단보도에 제한한다.
  범용/정적 회피에서 해당 자산을 빼는 것이 아니라 그곳에서는 정지 상태로 배치한다.
- NPC 차량과 이동/정지 외관 차이가 미관측된 자산은 single-frame OD용 정지 배치 우선이다.
  실주행 tracker의 동적 분류·속도 정책을 바꾸지 않는다.
- GPS 음영 끼어들기/객체 미션은 카메라 조명·배경 차이를 고려해 나중에 별도 저격 패밀리로 설계한다.
  이번 사거리 높이 보정표를 음영 구간에 사용하지 않는다.
- 이동 객체가 위 두 구역에만 나온다는 것은 사용자 수집 전략의 가정이다.
  MORAI가 확인한 전체 F2 에셋 등장 가능성과 별개이며 공식 출제 제한으로 기록하지 않는다.
- 일반 사람·Dog1/2·Deer1·Ped_Pet1은 이동 자세 변화 근거가 있다.
  BabyStroller2/3 및 Bike3/Scooter1은 기존 template의 높이를 맞춘 비교에서
  뚜렷한 외관 변화가 미관측됐다. 다른 유모차/탑승자/이륜차는 미검증이므로 같은 결론으로 묶지 않는다.
  미검증 자산의 별도 외관 검사와 필요한 추가 pivot 보정은 후속 사항이다.

## MR에 넣은 10개와 저장 이름

MR 시나리오 폴더:

```
~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025
```

| 번호 | 횡단보도 | 객체 | 불러올 파일 | 수동 보정 후 저장 |
| --- | --- | --- | --- | --- |
| 1 | CW000002 / 진행 순서 첫 번째 | Woman2 | 0_1_Woman2_CW000002.json | 1.json |
| 2 | CW000002 | Dog1 | 0_2_Dog1_CW000002.json | 2.json |
| 3 | CW000002 | Dog2 | 0_3_Dog2_CW000002.json | 3.json |
| 4 | CW000002 | Deer1 | 0_4_Deer1_CW000002.json | 4.json |
| 5 | CW000002 | Ped_Pet1 | 0_5_Ped_Pet1_CW000002.json | 5.json |
| 6 | CW000000 / 진행 순서 두 번째 | Woman2 | 0_6_Woman2_CW000000.json | 6.json |
| 7 | CW000000 | Dog1 | 0_7_Dog1_CW000000.json | 7.json |
| 8 | CW000000 | Dog2 | 0_8_Dog2_CW000000.json | 8.json |
| 9 | CW000000 | Deer1 | 0_9_Deer1_CW000000.json | 9.json |
| 10 | CW000000 | Ped_Pet1 | 0_10_Ped_Pet1_CW000000.json | 10.json |

각 시나리오에 actor1개, UNIQUEID=1, Ego 초기 속도0/Park이다. 기존 traffic spawn은 비웠다.
Ego는 해당 횡단보도 약20m 이전의 실제 route 위에 두고 최종 footprint의 MGeo road 지지를 확인했다.
이 검사는 live 정지·접지·CollisionData 인증이 아니다. 현재 MR을 자동 로드/이동시키지 않았다.
기존 ROS/SaveSensorData/CollisionData 네트워크 profile을 각 `EgoNetwork/<stem>_MN.json`에 함께 설치했다.
센서 설정과 현재 Sunny13시 환경은 이 도구에서 변경하지 않는다.

## 수동 작업 순서

1. 기존 연결과 Sunny13시를 유지하고 위 시나리오를 하나씩 불러온 뒤 F2 편집 모드에서 작업한다.
   이동 중 위치를 그대로 저장하지 말고 의도한 출발점과 경로를 기준으로 편집한다.
2. 객체 종류를 유지하고, 객체의 출발 위치를 UI에서 도로/보도 지면에 다시 배치한다.
   생성된 경로는 횡단보도 축을 따라 world X가 증가하는 쪽으로 진행한다.
3. 경로의 출발·중간3개·도착, 총5개 지점을 **UI에서 지면에 맞춰 다시 지정**한다.
   목표점만 고치면 나머지 경로 높이를 알 수 없으므로 경로 전체를 확인한다.
   UI에서 경로 재생성이 필요하면 그 방법을 사용해도 된다. waypoint 개수/XY 변경은 허용하고 실제 저장점을 읽는다.
4. 객체가 바닥에 닿아 있고 이동 경로도 맞는지 확인한 뒤 위 번호 그대로 `1`…`10`으로 저장한다.
   보정 전 `0_…` 파일을 덮어쓰지 않는다. 네트워크 sidecar도 생성되면 그대로 둔다.
5. 완료했거나 일부 번호만 완료했으면 에이전트에 알려준다. 재료가 부족한 번호를 성공으로 간주하지 않는다.

가까운 CW000002의 중심은 world 약(-61.974, -27.402), 먼 CW000000은 약(-62.211, -11.561)이다.
5개 지점은 각 중심에서 횡단보도 축으로 -12/-6/0/+6/+12m다. 양 끝은 횡단보도 바깥까지 포함하므로
보도·경계와 만나는 곳은 실제 UI 지면에 맞추며, 벽이나 접근 불가능한 지점이면 경로를 안전하게 줄이고 알린다.
사람/복합체 목표1.1m/s, 동물1.2m/s로 설정했다. 별도 animation Walk/Run 선택을 인증한 것은 아니다.

**JSON 축 주의:** actor의 `pos.z`는 world 높이다. 하지만 waypoint는 내부 좌표로 저장되어
`waypoint.pos.y`가 높이다. world→waypoint는 (-x, z, -y)이다. JSON의 모든 z만 바꾸면 경로가 망가진다.
UI 지면 배치가 waypoint 높이까지 자동 보정해 준다고 아직 가정하지 않는다. 저장 결과와 live 이동으로 확인한다.

## 수동 결과를 받으면 수행할 단계

1. 설치 manifest의 파일 이름·hash와 대응하는 보정본만 수집한다. 원본/보정본 및 각각의
   `EgoNetwork/*_MN.json`을 별도 backup으로 복사·hash 검증한 뒤 MR에서 해당 파일만 밖으로 옮긴다.
   이름이 숫자라는 이유로 다른 사용자 파일을 일괄 이동하지 않는다. 기존 bulk raw는 변경하지 않는다.
2. actor DataID/출발 XY·Z와 waypoint의 world XYZ를 비교한다. 저장된 pose와 initPos/standardPos의
   일치, 경로 실제 범위, 바닥 위치와 자산 pivot 기준을 확인한다. 경로 편집으로 GT를 사후 조작하지 않는다.
3. 실제 저장점을 횡단보도 기준 (진행 s, 횡방향 l, 높이 z)로 표현한다.
   사용자가 비스듬히 지정했더라도 실제 XY와 Z를 함께 사용한다. XY만 정렬해 높이가 그대로 유효하다고 가정하지 않는다.
4. 처음에는 **횡단보도×자산별 waypoint/pivot 높이 보간표**를 만든다.
   지면 높이와 자산의 pivot offset이 분리되지 않으면 이를 지면 mesh lookup이라고 부르지 않는다.
   다른 사람/복합체로 적용하려면 별도 offset 확인 또는 짧은 live 검증이 필요하다.
5. 보간은 실제 샘플 경로 주변의 검증된 좁은 띠에서만 사용한다. 단일 선 샘플은 2D 지면 전체를 결정하지 못한다.
   평행 경로·역방향·걷기/달리기 자세·자산 간 공유는 확인된 범위만 확대한다.
   경로 밖에는 미측정으로 반환하고 추가 수동 경로/기존 검사를 사용한다. 무제한 평면 외삽은 하지 않는다.
6. 실제 이동·Z span·LiDAR/카메라 접지·CollisionData·Ego 정지 및 GT 보존을 소량 검증한다.
   통과한 자산/경로에서만 매 장면 terrain/static-reference 준비를 lookup으로 대체하고 소요 시간을 다시 측정한다.
   경로가 길어지면 lookup이 **출발부터 도착까지** 덮어야 한다. 예컨대2.2m/s×9초의
   backward start19.8m는 현재 ±12m 샘플 밖이다. 측정 없이 그 구간을 flat으로 처리하지 않는다.
7. 범용/미션 quota를 유지하면서 범용 이동을 정지 캡처로 바꾼 새 manifest와 실행 계약을 동결한다.
   과거4171정적/829이동은 이전 manifest의 수량이며 변경 뒤의 수량/ETA로 그대로 인용하지 않는다.
   기존 output resume에 새 정책을 혼합하지 않는다.

## 생성·검증 근거

```
python3 dataset/morai_od/scripts/prepare_base5_intersection_manual_ground.py \
  --output "$HOME/morai_cap/base5_v2_261008/intersection_manual_ground_v116" \
  --install
```

이미 실행했다. 재실행은 기존 파일 보호를 위해 거부한다. 현재 MR에10개 JSON+10개 sidecar가 설치됐다.
기존 시나리오를 삭제·이동하거나 로드하지 않았다. 원본과 대응 좌표/자산/hash는
`~/morai_cap/base5_v2_261008/intersection_manual_ground_v116/calibration_manifest.json`에 보존한다.
이 파일은 수집 campaign manifest가 아니며 bulk runner에 전달하지 않는다.
단위검사6개: 좌표 왕복·정규화/constant Z·Park·5-waypoint 직렬화·기존 파일 보호·sidecar exact copy PASS.
Base5 전체 회귀검사278개 PASS(58.004초), 설치된10개 scenario/10개 sidecar hash 대조 PASS다.
보정본1.json…10.json은 아직 없으며 기존 사용자 파일은 덮어쓰지 않았다.
