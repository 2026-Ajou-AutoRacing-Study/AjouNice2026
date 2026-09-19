# 260718 FAST-LIO GNSS 실행 결과 차이 원인과 최종 해결 요약

## 한 줄 결론

첫 수정은 **매번 들어오는 GNSS correction의 시각**만 맞췄고 **첫 GNSS 원점의
시각**은 맞추지 못했다. 첫 원점도 GNSS timestamp의 EKF 상태로 계산하도록
수정한 뒤, 가벼운 실행과 전체 녹화 실행의 공통 궤적이 완전히 같아졌다.

## 1. 맨 처음 발생한 종방향 오차

GNSS 측정은 LiDAR scan 종료 상태보다 평균 약 40.2 ms 과거 값이었다.
기존 코드는 서로 다른 시각의 위치를 비교했다.

```text
기존 residual
= GNSS 위치(약 40 ms 전)
- FAST-LIO 예측 안테나 위치(현재 LiDAR scan 종료 시각)
```

차량이 전진하는 동안 과거 GNSS 위치와 현재 예측 위치를 비교했으므로, 측위
결과가 진행 방향으로 뒤처지는 것처럼 보였다. 이 차이는 고정 오프셋이 아니라
차량 속도에 비례하는 시간 불일치 오차였다.

## 2. 첫 번째 수정

IMU propagation 도중 GNSS 측정 timestamp에 해당하는 예측 안테나 위치를
저장하고, 다음처럼 같은 시각끼리 비교하도록 수정했다.

```text
수정 residual
= GNSS 위치(t_gnss)
- FAST-LIO 예측 안테나 위치(t_gnss)
```

이 수정으로 종방향 오차는 크게 줄었다.

## 3. 그런데 `/tmp`와 사용자 실행 결과가 달랐던 이유

반복 correction은 시각이 맞았지만, 첫 GNSS 상대좌표 원점은 여전히 callback
안의 `kf.get_x()`로 계산했다.

```text
첫 GNSS 메시지 timestamp의 상태가 아니라
ROS가 callback을 실제로 실행한 순간의 최신 EKF 상태를 사용
```

가벼운 `/tmp` 실행과 point cloud·raw 센서·TF까지 기록한 사용자 실행은 CPU,
메모리, 디스크 부하가 달랐다. 그 결과 callback 실행 순서가 달라져 같은 입력
bag인데도 첫 GNSS 원점이 약 3.42 cm 달라졌다.

```text
경량 실행 원점: [-1.50191, -0.0203349, 1.03464]
전체 실행 원점: [-1.52743,  0.0023697, 1.02935]
```

두 FAST-LIO 출력은 첫 GNSS update 전까지 같았고, 첫 update 직후부터 달라졌다.
작은 초기 차이가 incremental LiDAR map과 이후 EKF 상태에 영향을 주면서 후반
궤적 차이로 확대됐다.

## 4. 최종 수정

첫 GNSS callback에서 EKF 최신 상태를 즉시 읽지 않는다.

1. 첫 GNSS 측정과 timestamp를 버퍼에 저장한다.
2. IMU propagation이 그 timestamp에 도달할 때 안테나 위치와 body 회전을 저장한다.
3. 그 동시간 상태로 GNSS 상대좌표 원점과 축 정렬을 초기화한다.
4. 첫 샘플은 원점 정의에만 사용하고 correction은 두 번째 GNSS부터 수행한다.

핵심은 callback이 **언제 실행됐는지**가 아니라 센서 측정이 **언제 발생했는지**를
사용하는 것이다.

## 5. 최종 검증

동일 raw bag을 다음 두 조건으로 실행했다.

- 경량 녹화: Ego와 최종 state 중심
- 전체 녹화: LiDAR raw, IMU, GPS, cloud, TF, GNSS 디버그 토픽 포함 약 1.1 GB

두 실행의 첫 GNSS 원점은 숫자 끝자리까지 같았다.

```text
timestamp: 107.973000000 s
origin:   [-1.493769849, -0.027259919, 1.036219187]
```

공통 1,946개 state 샘플 비교:

```text
timestamp 최대 차이: 0 s
위치 평균 차이:      0 m
위치 RMSE 차이:      0 m
위치 최대 차이:      0 m
```

전체 평가 결과:

```text
XY mean error:  0.104 m
XY RMSE:        0.126 m
XY max error:   0.548 m
XY final error: 0.150 m
drift:          0.047 %
```

## 6. 의미

이번 수정은 특정 bag에 맞춘 40 ms 상수 보정이 아니다. 비동기 센서 융합에서
측정값과 예측값의 timestamp를 맞추는 일반적인 처리다. 따라서 여러 센서와
노드가 동시에 실행되어 callback 순서를 통제하기 어려운 대회 환경에 기존
방식보다 적합하다.

다만 다음 문제는 별개로 남아 있다.

- 정지 시 velocity drift와 position jitter
- 과도한 누적 path length
- GNSS noise, dropout, outlier 및 재수신 대응
- 완전한 out-of-sequence measurement state rewind

첫 GNSS 원점 재현성 문제는 해결됐지만, 위 항목은 이후 실시간 대회 적용 전에
추가 검증해야 한다.
