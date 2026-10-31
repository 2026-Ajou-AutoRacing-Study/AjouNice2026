# K-City v72 제한속도 수정

v72는 v71에서 `speed_limit=58`인 road lanelet만 `59 km/h`로 변경한다. 교차로의 `30/45 km/h`와 고속주회로의 `150 km/h`는 유지하며, geometry, lanelet member, topology와 다른 태그는 변경하지 않는다.

## 변경 결과

| 속도 | v71 | v72 |
| ---: | ---: | ---: |
| 30 km/h | 14 | 14 |
| 45 km/h | 34 | 34 |
| 58 km/h | 1,126 | 0 |
| 59 km/h | 0 | 1,126 |
| 150 km/h | 32 | 32 |

전용 검증기는 v71과 v72의 모든 node와 way, relation ID 및 member를 비교하고, 1,126개 road lanelet의 `speed_limit` 외에는 어떤 태그도 달라지지 않았는지 검사한다.

- v72 SHA-256: `f0e4fcecdce112579e6cec9c902025c92448f48f4c770178f7be586ea11badc6`
