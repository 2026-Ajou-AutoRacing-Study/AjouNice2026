# 신호등 짧은 UNKNOWN 유예 — 운영 배포 정리

종류: HANDOFF / 2026-10-27. 현재 권위는 [CURRENT](CURRENT.md)다.

## 적용하는 조합

사용자 승인으로 v5 지도+CNN·직접 관측·허가1회는 유지하고, 실제 방향 허가 뒤
순수 판독 공백만 최대0.3초 이어준다. 새 R/Y나 색이 있는 모순은 이 유예로 숨기지 않는다.
기한은 마지막 실제 허가의 원본 이미지 시각과 처리 시각 **둘 다** 만족해야 한다.
반복 UNKNOWN·중복 stamp는 기한을 늘리지 않으며 다른 미션·시간 역전·stale도 제외한다.
황색 통과는 OFF, 3/2·7/4 투표는0, 예전 HSV/profile 램프 추가는 복원하지 않는다.

| 경로 | 판단 유예 | 인지 색 증거 보존 |
|---|---:|---|
| 일반 v5 + guard + traffic ON | 0.3초 | ON |
| 전체 스택 traffic OFF / guard OFF / planner plugin OFF | 0 | OFF |
| validator legacy | 0 | OFF |
| 새 필드 없는 과거 frozen 실행 재개 | 0 | OFF |

전체 스택 `morai_planning_stack.launch`와 새 validator 실행은 동일 설정을 양쪽에 전달한다.
새 실행은 resolved 값을 동결하고, 과거 snapshot/ledger는 고치지 않는다.
낮은 수준 Python/C++ 기본은 false/0을 유지해 새 launch 밖에서 정책을 몰래 켜지 않는다.
단독 planning launch와 단독 v5 launch의 운영 기본은 각각0.3/ON이다.

## 설치와 되돌리기

PR 병합 후 기존 planning 브랜치에서 pull하고 다음을 실행한다.

```bash
bash scripts/runtime/install_traffic_light_candidate.sh
bash scripts/runtime/install_traffic_light_candidate.sh --check
```

기존 모델 압축본·YOLO/CNN 가중치는 그대로다. C++ 신호등 모듈 재빌드와
traffic/validator/integration 증분 설치가 필요하며, 이미 켜진 스택은 바뀌지 않는다.
설치 후 스택을 재시작하고 validator는 **새 실험**을 만든다.

되돌리기는 전체 스택에 `signal_compliance_unknown_grace_sec:=0.0`, validator 새 설정에는
`stack.signal_compliance_unknown_grace_sec: 0.0`을 지정한다. marker도 자동 OFF다.
분리 실행이라면 planning에 위 유예0, traffic에 `preserve_unreadable_color_evidence:=false`를
함께 준다. 별도 legacy 인지 + 단독 planning 조합도 유예0을 명시한다.

## 근거와 남은 한계

[실제 주행·동일 입력 비교](261027_UNKNOWN_GRACE_DRIVING_COMPARISON.md)는 Sunny13·명시적v88·od_gt,
실제 ROS1 planner/PP·PI/localization의 자연 A/B/C/D 전체 주행이다. 충돌0·종료 정지/P를 확인했다.
해당 실행에서 A/B/C/D의 회복되는 내부 GO→UNKNOWN→GO 재정지 사건은 없었다.
과거 D 단절의 동일 입력 Guard 재생에서는 100~150ms STOP을0으로 줄였다.
단, 그 단절이 새 실제 주행에 재현되지는 않아 실제 움찔 개선의 인과·완치를 인증하지 않는다.
영상·bag·대형 결과는 로컬 `artifacts/evaluation/traffic_light_robustness_261025/`에 남고 Git에는 넣지 않는다.

새 적색이 가림 등으로 **색 근거조차 없이** 사라지면 이전 허가가 원본 age0.3초까지 남을 위험은 있다.
이는 7/4의 R/Y 전환 지연을 도입하지 않는 대신 제한적으로 택한 공백 완화다.
최신 planning 지도·팀4090·LAN·BEVFusion 동시 부하·공식 신호 패널티까지 인증한 것은 아니다.
과거 날짜별 증거와 현재 운영 설정·설치 receipt를 구분한다.
이번 PR에는 최신 planning 변경을 통합하며 운영 지도 기본은 v96이다. v88 시험은 통합 전
`6e663eea`까지의 코드·동결 설정·receipt에 대응한다. 최신 코드에서 `--map-file`만 v88로
바꾸면 route/mission 기본은 v96으로 남을 수 있어 그 명령만으로 역사 시험을 재현하지 않는다.

## 통합 후 확인

- 신호등 인지·평가 Python 497개, Native C++ 132개 통과.
- 실제 source/installed `roslaunch --dump-params` 각각13개 통과. launch 5개 SHA 일치.
- 딸깍 설치기 scoped rebuild/install 및 `--check` 통과. 기존 모델 재다운로드 없음.
- Validator 전체470개 중468개 통과. full-course waypoint16/17 및 mission 생성 자산의
  TF SHA 불일치2개는 통합 전/원격 planning에도 있는 기존 실패로, 자산을 재생성하지 않았다.
- 문서 검사에서 신호등 CURRENT100줄·링크는 유효하다. 통합한 원격의 지도 README85줄은
  기존65줄 예산을 초과하며 이번 신호등 배포에서 그쪽 문서를 임의 재작성하지 않았다.
- 이번 최종 설치 검사는 노드·주행을 시작하지 않았고 실행 중인 스택 설정도 바꾸지 않았다.
