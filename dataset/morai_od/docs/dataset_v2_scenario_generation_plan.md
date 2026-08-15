# MORAI OD Dataset V2 시나리오 생성 계획

## 목적

기존 Sunny13 1,500-frame 데이터는 Fast-BEV 파이프라인과 첫 baseline을
검증하는 데 충분했지만, 중·원거리와 다양한 obstacle 형상을 학습하기에는 작다.
Dataset V2는 asset을 삭제해 쉬운 분포만 만드는 대신, 서로 다른 scene과 관측
조건을 늘려 `vehicle`, `pedestrian`, `obstacle` 세 class의 일반화를 높인다.

첫 단계에서는 기존 데이터로 사전학습·모델 초기화 실험을 끝낸다. 그 결과와 오류
분석을 바탕으로 2,000-3,000 frame 규모의 V2 pilot을 수집하고, 개선이 확인될
때만 총 8,000-12,000 frame으로 확장한다.

## 장면 구성

완전한 단일-class 장면만 만들지 않고, 현실적 혼합 장면과 class-focused 장면을
함께 사용한다.

| 장면 유형 | 목표 비율 | 기본 객체 구성 |
|---|---:|---|
| 현실적 혼합 | 40% | 차량 2-5, 보행자 1-4, 장애물 1-3 |
| 차량 중심 | 20% | 차량 5-8, 다른 class 0-2 |
| 보행자 중심 | 15% | 보행자 6-10, 다른 class 0-2 |
| 장애물 중심 | 20% | 장애물 5-8, 다른 class 0-2 |
| 희소·빈 장면 | 5% | 전체 객체 0-1 |

집중 장면 중 약 1/3은 목표 class만 배치하고, 약 2/3에는 다른 class를 0-2개
섞는다. 이 방식은 각 미션 객체를 충분히 학습시키면서도, `보행자가 있으면 차량은
없다` 같은 비현실적 장면 prior와 다중 class 가림·오검출 문제를 줄인다.

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
우선한다. V2 pilot은 scene당 2-3 frame을 기본으로 하며, temporal 모델용 연속
sequence는 별도 수집 정책으로 관리한다.

- single-frame Fast-BEV: 서로 다른 배치와 시점 다양성 우선
- 향후 StreamPETR: 일정 주기, ego pose, timestamp가 연속된 sequence 단위 수집
- 동일 scene의 연속 frame과 환경 variant는 동일 split에 유지
- split은 frame이 아니라 base scenario/anchor 단위로 결정

## 단계적 실행

1. 기존 1,500-frame 데이터에서 ImageNet, nuImages, nuScenes 초기화 실험을 끝낸다.
2. 오류 분석을 반영한 2,000-3,000-frame V2 pilot을 생성·수집한다.
3. 기존 모델과 동일한 설정으로 data-scale 효과만 비교한다.
4. class·형상·거리별 개선이 확인되면 8,000-12,000 frame으로 확장한다.
5. 새 metadata에는 asset 이름, camera별 visibility와 projected 2D area를 보존한다.

Dataset V2의 목적은 쉬운 obstacle만 남겨 지표를 높이는 것이 아니라, 대회에서
발생 가능한 객체 분포를 유지하면서 특히 10-30 m recall과 center/size 정확도를
높이는 것이다.
