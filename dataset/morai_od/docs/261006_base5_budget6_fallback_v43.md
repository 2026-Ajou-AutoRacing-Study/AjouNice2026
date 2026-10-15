# Base5 v43 — 자연 이동 준비 6초 후보와 안전한 9초 재시도

2026-10-06 / EVIDENCE. 본10K 캡처·학습은 시작하지 않았다.
Canonical v35 10K와 기본 준비 여유9초는 그대로다. Production 계약도 미변경이다.

## 실제 MR 검증

- 다양한 자연 이동16셀: 14종 primary asset, 걷기/달리기·Dog2·Deer1·Ped_Pet1 포함.
  전방7/좌6/우2/후방1, 거리3–30m. 최종16/16, 실제19촬영.
  A8셀91.65초/10촬영, B8셀79.03초/9촬영. 가시성 실패2셀은 동일 quota 보충으로 통과했다.
  CPU train12/val4/test0, GT23. 정상 trial에는 목표 도착 실패가 없었다.
- 동일 대표 혼합32셀의6초 후보: 32/32, 실제36촬영, wall216.00초
  (driver215.892706초). Event87회, 짧은 handler 응답 오류4회는 기존 retry로 회복,
  timeout/미종료 worker0. 모든 boundary/final Park 확인.
  CPU train26/val6/test0, GT48(vehicle21/pedestrian10/obstacle13/animal4).
- 이전9초 경로233.02초 대비 wall 약7.3% 감소. 동일 family 가중 방식의10K 외삽은
  v36 27.24 → v42 20.19 → v43 **18.71시간**이다.
  소량 coverage 표본·순차 실행의 예상이며 신뢰구간/완료 보장이 아니다.
  목표 도착 실패 재시도 비용은 정상 표본에 없으므로 외삽에도 반영되지 않았다.

## 실패를 숨기지 않는 재시도

초기6초 경로에서 모든 actor의 목표 도착을 관측하지 못한 경우에만9초를 한 번 재시도한다.
캡처 미요청·토픽 신선도·actor 존재·Ego 정지·actor Z 연속성·CollisionData 무접촉·Park와
원본 session/시나리오/실패 증거 SHA를 확인한다. 이 증거는 지면 인증이 아니다.
9초 새 캡처도 원래 지면/Z/거리/visibility gate를 모두 통과해야 한다.
통신·충돌·Park 실패, 보완 후 목표 실패,9초 재시도 실패는 중단한다.
실패한6초 source batch를 resume해 같은 실패를 반복하지 않으며 남은 chunk tail을 분리한다.

실제 fault 진단에서는 서비스 발견 직후 준비를 한 번5초 지연했다.
6초 경로는 목표를 놓쳐 **센서 캡처 없이 안전 실패**,9초 재시도는 정상 동작/지면/Z/센서
검사를 통과했다. 전체43.01초. 양쪽 Park 확인.
진단 metadata는 캡처 전에 calibration_only로 표시했다.
실제 변환 결과 train/val/test=0/0/0, GT0: 학습 quota에 섞이지 않는다.
이는 artificial 지연 복구 실험이지 실제 실패율 추정이 아니다.

## 산출물과 다음 단계

Base5 단위 테스트137개 통과, diff whitespace 검사 통과.

공통 root: /home/luv/morai_cap/base5_v2_261006/

- budget6_diverse_a_v43 / budget6_diverse_b_v43: 각8셀 OFFLINE_PASS.
- budget6_diverse_a_capture_v43 / budget6_diverse_b_capture_v43: 완료 raw/audit.
- budget6_diverse_a_infos_v43 / budget6_diverse_b_infos_v43: CPU 변환.
- budget6_mixed32_v43: 32셀 OFFLINE_PASS.
- budget6_mixed32_capture_v43 / budget6_mixed32_cost_v43.json: 완료 및 비용.
- budget6_mixed32_infos_v43: 최종32장 변환.
- budget6_late_diagnostic_v43 / budget6_late_diagnostic_infos_v43: 진단 전용, 학습 제외.

8077 dashboard는 혼합32셀 완료 결과를 표시한다. collector 코드 hash가 변경된 과거
campaign은 강제로 resume하지 않는다. 원본 raw는 읽기 전용 근거로 보존한다.
다음은6초 후보의 더 넓은 자산/후방/복합 조건과 장시간 안정성 확인 후 기본값 채택 판단이다.
이번14종/후방1셀로 전체 asset pool을 인증하지 않는다. 본수집은 별도 승인 뒤 진행한다.
