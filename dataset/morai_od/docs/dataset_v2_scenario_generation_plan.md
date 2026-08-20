# MORAI OD Dataset V2 시나리오 생성 계획

## 목적

기존 Sunny13 1,500-frame 데이터는 Fast-BEV 파이프라인과 첫 baseline을
검증하는 데 충분했지만, 중·원거리와 다양한 obstacle 형상을 학습하기에는 작다.
Dataset V2는 asset을 삭제해 쉬운 분포만 만드는 대신, 서로 다른 scene과 관측
조건을 늘려 `vehicle`, `pedestrian`, `obstacle` 세 class의 일반화를 높인다.

첫 단계에서는 기존 데이터로 사전학습·모델 초기화 실험을 끝낸다. 그 결과와 오류
분석을 바탕으로 2,000-3,000 frame 규모의 V2 pilot을 수집하고, 개선이 확인될
때만 총 8,000-12,000 frame으로 확장한다.

## 최종 10,000-frame 구성

완전한 단일-class 장면만 만들지 않고, 현실적 혼합 장면과 class-focused 장면을
함께 사용한다. 다만 대회의 미션별 객체 분포와 일반화용 혼합 분포를 구분한다.

- 미션 중심 장면은 실제 대회 구조를 보존한다. 로터리·고주로에는 차량,
  횡단보도 동적 장애물에는 보행자, 정적 장애물 구간에는 obstacle을 주 대상으로
  두며 관계없는 동적 class를 억지로 섞지 않는다.
- 일반화 장면은 여러 class가 같은 시야에 자연스럽게 존재하도록 섞는다. 이는
  `횡단보도이면 pedestrian만 존재한다` 같은 위치·문맥 shortcut을 막고,
  구간 전환부와 가림 상황의 오검출을 줄이기 위한 것이다.
- 따라서 데이터셋을 혼합한다는 것은 모든 scene에 모든 class를 넣는다는 뜻이
  아니다. 특히 로터리·고주로·횡단보도 핵심 미션에 관계없는 obstacle을 임의로
  추가해 미션의 pure한 상호작용을 깨뜨리지 않는다.

| 상위 구성 | 목표 비율 | 역할 |
|---|---:|---|
| 미션 중심 clip | **약 50%, 5,004 frame** | 로터리·고주로 차량, 횡단 보행자와 미션 obstacle prior 보존. |
| 정적 random | **35%, 3,500 frame** | asset·형상·거리 다양성과 정적 obstacle 표본 확대. |
| hard case | **15%, 1,500 frame** | 가림, camera seam, 원거리, 속도 변화와 희소 배경. |

첫 10,000-frame 학습에서는 `structured-random dynamic`을 **보류**한다. 이
family가 불필요하다는 결론은 아니다. 정적 obstacle 다양성은 서로 다른 scene을
많이 만드는 정적 random이 temporal clip보다 효율적이고, 첫 velocity regression과
temporal OD 파이프라인은 현재 미션 dynamic만으로 검증할 수 있기 때문이다. 첫
학습에서 일반 도로의 동적 운동·다중 class 문맥 shortcut이 실제 문제로 확인될
때만 10~15% pilot을 추가하고 이후 최대 20~27%까지 검토한다.

미션 중심 약 50%는 실제 대회처럼 family별 주 객체를 분리한다. 로터리·고주로에는
차량, 횡단보도에는 보행자, 정적 장애물·GPS 음영 obstacle mission에는 obstacle을
중심으로 두며, 각 family에 관계없는 obstacle을 추가하지 않는다.
다중 class 일반화는 정적 random의 현실적 혼합 장면으로 담당한다.

### 1,500-frame 단계 수집

대용량 1회 수집 대신 약 1,500 frame씩 검증하면서 확장한다. 각 full batch는 최종
비율을 유지한다.

| 한 batch 구성 | 목표 frame | scenario/clip 환산 |
|---|---:|---:|
| 미션 중심 | 744 | 12-frame dynamic clip과 3-frame static mission scene의 조합. |
| 정적 random | 525 | 3-frame scene 175개. |
| hard case | 231 | 정적 3-frame scene과 동적 12-frame clip의 별도 quota. |
| 합계 | 1,500 | 실패율을 고려해 생성 manifest는 5~10% 여유를 둔다. |

`744 + 525 + 231 = 1,500`은 scene/clip을 자르지 않는 clip-safe 할당이다.
1,500-frame batch 6개로 9,000 frame을 확보한 뒤 마지막 999-frame batch를
`540 mission + 351 static random + 108 hard case`로 구성한다. 최종 합계는
9,999 frame이며, 숫자 10,000을 맞추려고 temporal clip을 분할하지 않는다. 목표는
생성 요청 수가 아니라 artifact 완전성 검증을 통과한 accepted frame 수다. 실패
scene을 다른 split의 frame으로 메우지 않고 같은 family·split의 예비 scene으로
대체한다.

모든 temporal frame은 clip 단위로 같은 train/val/test split에 둔다. 동일 anchor,
동일 actor path, 같은 base scenario의 variant도 split을 넘나들지 않는다.

### 미션 vehicle의 motion-phase 비율

공식 sample의 spawn 가속은 T&P production의 주 상황은 아니지만 OD와 velocity
regression에는 유용하므로 삭제하지 않고 별도 family로 제한한다.

| 구간 | spawn 가속 | production 대표 상태 |
|---|---:|---:|
| 로터리 vehicle clip 내부 | 10~15% | 안정 순환·Ego 합류·출구 85~90% |
| 고주로 vehicle clip 내부 | 15~20% | 합류 순항·following·추월 80~85% |

- spawn 가속은 `motion_phase=spawn_acceleration`과
  `sampling_role=minority_hard_case`로 기록한다.
- 목표속도 시작과 순항 상대운동은 production representative로 기록한다.
- 10,000-frame 전체에서 `highway_merge_sample_spawn`은 약 300~500 frame을
  권장하며, 실제 수집 결과의 actor visibility가 낮으면 더 늘리지 않고 anchor를
  다시 잡는다.
- T&P 모델 선택은 CV-stable을 주 지표로 유지하고 spawn 구간은 별도 slice로
  보고한다. OD는 두 slice 모두 학습하되 scenario-family sampling weight를 적용한다.

### 정적 random 35% 내부 구성

| 장면 유형 | 목표 비율 | 기본 객체 구성 |
|---|---:|---|
| 현실적 혼합 | 35% | 차량 2-5, 보행자 1-4, 장애물 1-3 |
| 차량 중심 | 10% | 차량 5-8, 다른 class 0-2 |
| 보행자 중심 | 10% | 보행자 6-10, 다른 class 0-2 |
| 장애물 중심 | **40%** | 장애물 5-8, 다른 class 0-2 |
| 희소·빈 장면 | 5% | 전체 객체 0-1 |

Obstacle 중심 비율을 높인 이유는 이전 1,500-frame 데이터에서 전체 frame 수보다
개별 obstacle asset이 등장한 서로 다른 scene 수, 원거리·측면·가림 조건이
부족했기 때문이다. 동적 clip에 같은 obstacle을 12번 반복하는 대신, 3-frame 정적
scene의 asset·pose·anchor를 더 자주 바꾼다.

집중 장면 중 일부는 목표 class만 배치하고 나머지에는 다른 class를 0-2개 섞는다.
현실적 혼합 35%가 미션 clip을 오염시키지 않고도 `보행자가 있으면 차량은 없다`
같은 비현실적 class prior와 다중 class 가림·오검출 문제를 줄인다.

## Obstacle 표본 확대 원칙

Obstacle은 단순 개수보다 형상·거리·관측 조건의 quota를 우선한다. 대회에 나올
가능성이 있는 asset을 성능을 위해 제거하지 않는다.

- 형상: compact / tall / long-thin
- 거리: 0-10 / 10-20 / 20-30 / 30 m 이상
- 방향: 정면 / 측면 / 대각선
- 가림: 없음 / 부분 가림 / 심한 가림
- 시야: front / front-left / front-right / back 및 camera seam
- 크기: 소형 / 중형 / 대형
- 문맥: 도로, 인도 경계, 주차 공간, 구조물 근처

V2 본 수집 목표는 각 compact/tall/long 형상에 대해 0-10 m와 10-20 m를 각각
최소 2,000 instance, 20-30 m를 최소 1,000 instance 확보하는 것이다. 총 frame
수보다 서로 다른 asset, anchor, pose, occlusion과 scene 수를 우선한다.

## 프레임과 시나리오 다양성

동일 scene에서 연속 6 frame을 늘리는 것보다 서로 다른 scene을 늘리는 편을
우선한다. 정적 random 세트는 scene당 3 frame을 `3.0 m` 간격으로 촬영한다.
trigger는 시작점에서 약 3, 6, 9 m이고, 기존 `6 frame × 1.5 m`와 같은 약 9 m
공간을 더 낮은 프레임 상관성으로 덮는다. 안전 회랑과 최대 허용 이동거리
`10.5 m`는 느슨하게 하지 않는다. temporal 모델용 연속 sequence는 별도 수집
정책으로 관리한다.

- single-frame Fast-BEV: 서로 다른 배치와 시점 다양성 우선
- 향후 StreamPETR: 일정 주기, ego pose, timestamp가 연속된 sequence 단위 수집
- 동일 scene의 연속 frame과 환경 variant는 동일 split에 유지
- split은 frame이 아니라 base scenario/anchor 단위로 결정
- 현재 파이프라인 완성용 1,998-frame 정적 세트는 서로 다른 666 scene을 모두
  `Sunny 13:00`에서 수집한다. 날씨·시간 일반화는 수집·학습·온라인 추론
  파이프라인이 완성된 뒤 별도 단계로 확장한다.

## 단계적 실행

1. 기존 1,500-frame 데이터에서 ImageNet, nuImages, nuScenes 초기화 실험을 끝낸다.
2. 오류 분석을 반영한 2,000-3,000-frame V2 pilot을 생성·수집한다.
3. 기존 모델과 동일한 설정으로 data-scale 효과만 비교한다.
4. class·형상·거리별 개선이 확인되면 8,000-12,000 frame으로 확장한다.
5. 새 metadata에는 asset 이름, camera별 visibility와 projected 2D area를 보존한다.

Dataset V2의 목적은 쉬운 obstacle만 남겨 지표를 높이는 것이 아니라, 대회에서
발생 가능한 객체 분포를 유지하면서 특히 10-30 m recall과 center/size 정확도를
높이는 것이다.
