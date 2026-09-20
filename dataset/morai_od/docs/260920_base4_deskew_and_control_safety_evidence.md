# Base4 LiDAR deskew 및 고주로 제어 안전성 근거

- 상태: **CURRENT EVIDENCE**
- 날짜: 2026-09-20
- 적용 범위: Base4-Temporal 고주로 수집기

## 결론

1. MORAI PointCloud2의 point `time`은 단순 장식 field가 아니다. 정상 scan에서는
   point-time Ego deskew가 정적 구조물의 frame 간 정합을 개선했다.
2. 첫 scan 일부는 point-time span이 `1 s`를 넘는 비정상 값이다. `0.15 s`를 넘는
   scan은 deskew 입력에서 제외해야 한다.
3. scan 내부 deskew는 아직 production 기본값으로 켜지 않는다. raw를 보존하고,
   Base4 converter에서 optional A/B로 구현한 뒤 학습과 online inference에 같은
   계약을 적용한다.
4. 최초 고주로 production `p12v3`는 제어권 인수 전에 움직이는 Ego 때문에 ramp 차선을
   이탈했으므로 중단·격리했다. 해당 데이터는 학습에 사용하지 않는다.
5. Ego 초기속도 `0`, pre-control drift gate, 실제 속도/안정화 capture gate, NPC 만남
   시점 역산을 적용한 `r9/r10` smoke는 merge와 70/100 km/h cruise를 통과했다.

## Deskew P0

재현 도구:

```bash
python3 dataset/morai_od/scripts/analyze_morai_lidar_deskew.py --help
```

결과 파일:

```text
artifacts/datasets/base4_temporal_deskew_p0/
  highway_merge_deskew_ab_stratified40.json
```

40 bag, 1,911 cloud, 1,631 consecutive pair를 사용했다. 차량 actor point와 ground를
제외하고, Ego pose를 보간해 정적 world alignment를 비교했다.

| 항목 | 결과 |
|---|---:|
| point-time span p50 / p95 | `0.08149 / 0.09903 s` |
| 최대 span | `1.39206 s` |
| `0.15 s` 초과 scan | `42 / 1,911` |
| zero-time median alignment p50 | `0.22819 m` |
| point-time deskew median alignment p50 | `0.20640 m` |
| reverse-time median alignment p50 | `0.25280 m` |

정방향 point-time 보정은 zero-time 대비 median p50을 약 `9.5%` 개선했고, 반대 방향
보정은 악화시켰다. 따라서 MORAI scan은 완전한 동시 snapshot으로 간주하면 안 된다.
다만 평균 개선은 `0.36660 -> 0.35950 m`로 작고 outlier scan이 있으므로, 이 결과만으로
production deskew를 강제하지 않는다.

## 차선 이탈 원인과 수정

격리한 `p12v3` 4,656 frame에서 조향 절댓값은 p50 `10.328°`, p95/max `12.605°`였고
`45.2%`가 제한에 포화됐다. 원인은 ramp route가 아니라 다음 순서였다.

1. 시나리오가 저장된 45/50/54 km/h로 즉시 출발한다.
2. ScenarioLoad 후 ROS network와 external-control service가 복구되는 동안 steering command가
   없다.
3. Ego가 곡선 진입부를 직진으로 통과한다.
4. 뒤늦게 controller가 붙어 최대 조향으로 route를 복구하려 한다.

수정 계약:

- Ego load speed `0 km/h`.
- external control 전에 기대 위치에서 `0.75 m` 넘게 이동하면 ScenarioLoad 재시도.
- capture 전에 motion arm, 최소 Ego speed, stable-speed window를 모두 확인.
- post-merge 속도는 우선 `70/85/100 km/h`; `115/120`은 기준선에서 제외.
- NPC가 load 직후부터 이동하는 거리를 Ego 가속시간으로 역산해 spawn 위치를 보정.

## R9 실제 smoke

경로: `/home/luv/morai_cap/260920_b4_r9_smoke`

| 구간 | 완료 | 상대차 거리 | 조향 abs p95 / max | 실패 |
|---|---:|---:|---:|---:|
| merge 50 km/h | 16/16 | `17.6--20.8 m` | `0.282 / 0.284°` | 0 |
| cruise 70 km/h | 16/16 | `29.6--42.3 m` | `0.093 / 0.100°` | 0 |
| cruise 100 km/h | 16/16 | `11.6--39.0 m` | `0.230 / 0.240°` | 0 |

초기 직진 이탈, 조향 포화, collision, actor-gate timeout은 재현되지 않았다.
100 km/h는 stable-speed gate가 약 176 m에서 열리므로 16-frame window를 보존하도록
maximum route motion을 `260 m`로 둔다.

## 후방 접근 차선과 lane change

인접 차선 후방 차량의 velocity 관측은 필요하지만, 이를 곧바로 Ego cut-in과 묶지 않는다.

- 1차 Base4 본분포: Ego는 현재 차선을 유지하고 옆 차선 후방/사선 차량을 관측한다.
- 후속 addon: 실제 lane change를 소량만 추가한다.
- addon 안전 gate: 예상 TTC `>= 4--5 s`, 후방 초기 간격 `>= 30 m`, restricted lane 제외.
- NPC 목표속도 `40/50/60 km/h`를 유지하면, 고속 Ego보다 뒤차가 접근하는 장면은 Ego가
  느린 선행차 속도에 맞춘 추월 준비 단계에서 만든다.

실제 cut-in을 첫 수집에 섞으면 controller 실패와 velocity supervision 실패를 구분하기
어려우므로 별도 campaign으로 유지한다.
