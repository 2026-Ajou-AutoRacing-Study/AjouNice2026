# Base5 Sunny 11시 직접 5K — 최신 수집 목표

2026-10-07. 사용자가 이번 수집을10K→5K로 변경했다.
**10K를 수집한 뒤 절반 선별하는 방식이 아니라, 처음부터 균형5K를 설계한다.**
본수집은 아직 시작하지 않는다. 기존10K manifest/raw/pilot은 삭제하지 않는다.

## 구성

| 패밀리 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 |900 |600 |1,500 |
| 로터리 차량 |100 |250 |350 |
| 기타·정적 차량 |100 |50 |150 |
| 실체 장애물 |900 |600 |1,500 |
| 보행자·동물·복합 |1,000 |250 |1,250 |
| 빈 배경 |250 |0 |250 |
| 합계 |3,250 |1,750 |5,000 |

범용positive3,000장: 단일900/일반혼합1,500/밀집혼합600.
배경250장은 별도다. 크기 비용·10–14/16–22점·최대8개·가시성 하한은 유지한다.
미션은 상황별1–3개 정책을 유지하며generic예산으로객체를더채우지않는다.
보행자 관련은person938/dummy62/animal63/pet_person62/stroller62/rider63으로
합계1,250장을맞춘다. 홀수quota반올림의차이는최대1장이다.

## 설정 및 변경 범위

- 새 기본config: `dataset/morai_od/configs/base5_independent_5k_sunny11.json`.
- `generate_base5_independent.py`의기본CONFIG도새5K로변경한다.
- 구 `base5_independent_v2.json`은10K재현용으로보존한다.
- weather/hour는수집의도metadata이며MR실제설정자동적용·인증을뜻하지않는다.
  live전Sunny11시실제세팅을확인한다.
- 4class/scale1/단일정지Ego/필요자산실제애니메이션/GT·접지·충돌·resume계약은유지한다.
- 기존output에새config를섞어resume하지않는다. 최종manifest/output은새경로로만든다.

## 남은 보완

기존[5개gate계획](261007_base5_final_candidate_five_gates_plan.md)의1번population구현은
유지한다. 2번범용·회피초근접,3번고주로·로터리밀집,4번GT대표검증은남는다.
5번은A/B각5K선별이아니라 **직접5K의균형·희귀자산유효coverage감사**로대체한다.
5K축소는희귀정적차량의장수를더줄일수있다. 자산종류coverage와각자산이실제로
보이는횟수는별개이며,추후primary/secondary관측하한을검토해야한다.
최종6환경×5K=30K목표는유지하되이번수집은Sunny11시5K만이다.

이번변경은config·spec전체quota·회귀검증까지다. 새5K시나리오전체생성/실캡처/학습
완료나본수집준비완료를뜻하지않는다.
