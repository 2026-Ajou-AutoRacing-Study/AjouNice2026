# SOA envelope 누적 제거 및 smoother 조향 레이트 감속 해제

## 변경

- velocity_smoother `enable_steering_rate_limit: false`. 소스 및 install 설정이 일치하며 두 격리 재생의 런타임 파라미터에서도 false 확인. 제어기 조향 레이트 제한은 그대로다.
- optimization_base SOA는 현재 관측에서 매번 envelope를 새로 만든다. upstream의 과거 위치 union과 downstream의 과거 최대 형상 병합을 모두 제거했다.
- 유효한 새 관측은 현재 형상을 교체한다. 커지거나 작아진 관측 모두 반영한다. 관측 누락/잘못된 갱신은 마지막 형상을 유지하며 기존 통과 후 해제 조건을 사용한다.
- 다른 envelope에 완전히 포함되고 hard/soft margin도 더 크지 않은 중복 제약은 해당 주기의 출력에서 생략한다. 객체 상태를 삭제하지 않아 포함 관계가 깨지면 다시 출력한다. 부분적으로 겹친 서로 다른 ID를 같은 물체라고 단정하여 지우지는 않는다.

## 국소 곡률

조향 레이트 감속을 끄기 위해 추가 국소 곡률을 계산할 필요는 없다. 해당 필터 자체가 호출되지 않는다. 기존 횡가속도 필터의 1m 국소 곡률 검사는 유지했다. 앞뒤 5m 점으로 계산하는 곡률은 짧은 S자 구간을 평균화할 수 있으므로 두 계산은 결과가 같지 않다. 이번 변경은 횡가속도 제한을 해제하지 않는다.

## 빌드와 테스트

- SOA CMake install target 빌드/설치 완료. 동적 로딩 플러그인으로 BPP 재빌드는 필요하지 않다.
- utils 테스트 24개 통과, 기존 비활성 테스트 1개. plugin discovery 테스트 1개 통과.
- 현재 관측 확대/축소, 누락/invalid 관측 유지, 30회 위치 흔들림에 대한 면적 비누적, 중복 제약 생략 및 복원 검증.
- 설정 source/install cmp 일치. git diff --check 통과.
- 실제 실행 중인 planning은 재시작해야 반영된다. 이 작업에서 실제 ROS 시스템은 재시작하지 않았다.

## 백 재생

private ROS master 11475에서 native player와 full planning pipeline 사용. 기록된 객체 및 ego를 입력했으며 폐루프 주행 검증은 아니다.

| 항목 | GT 20:44:17 | BEV 20:16:21 |
|---|---:|---:|
| PO 프레임 | 287 | 202 |
| 장애물 활성 PO 프레임 | 252 | 143 |
| PO ego 위치 정지 출력 | 0 | 72 |
| 최종 궤적 프레임 | 287 | 208 |
| 최종 ego 위치 정지 출력 | 0 | 74 |
| 최종 이동 궤적의 장애물 충돌 판정 | 0 | 0 |
| 관측 envelope+hard margin과 실측 차체 겹침 | 0 | 0 |
| validator invalid 필드 | 없음 | lateral jerk 1회, longitudinal min acc 1회 |
| PO 계산 시간 p50/p95/max, ms | 47.74 / 74.13 / 103.80 | 53.81 / 100.22 / 130.59 |
| PO 발행 간격 최대 상한, ms | 200.01 | 197.27 |

GT 12–24초 구간 최종 궤적의 ego 최근접점 목표 속도는 최소/중앙값/최대 18.88/19.74/23.46 km/h다. 이전 기록의 최소/중앙값 2.72/4.80 km/h 대비 조향 레이트 감속 영향이 제거됐다. 기록된 ego는 원래 저속 주행이므로 이 숫자는 새 속도로 실제 추종 가능함을 입증하지 않는다.

BEV의 정지는 남아 있다. 로그상 최신 장애물에 의해 이전 출력이 무효화되고 PO 재최적화가 실패하여 이전 경로 위에서 제동한다. 이후 이전 경로의 clear distance가 0이 되어 emergency brake가 반복된다. 이번 재생에서는 measured margin overlap 정지는 없지만 이것만으로 BEV 회피가 해결되었다고 판정할 수 없다.

BEV PO의 동일 scene stamp 검사에서는 이동 충돌 0건이다. observer가 마지막으로 받은 scene 기준 검사에서는 PO 2건, smoother 1건이 있으며, 최종 출력은 0건이다. 비동기 scene 갱신과 출력 도착 순서가 달라지는 검사이므로 두 수치를 구분한다. 최종 출력의 무충돌 판정도 검증 대상 관측/마진 내 기하 검사 결과다.

BEV의 이전 작은 ID 47과 차량 ID 49는 일부 프레임에서 여전히 둘 다 출력된다. 완전히 포함되지 않은 작은 돌출 부분이 있기 때문이다. 이를 무조건 삭제하는 ID 병합은 이번 변경에 포함하지 않는다. 같은 ID 내 시간 누적 형상은 제거했다.

## 결과 파일

- `planner_logs/analysis/soa_no_accum_204417/`: GT result.json, metrics.json, stage_metrics.json, runtime_smoother.json, full_pipeline.log
- `planner_logs/analysis/soa_no_accum_201621/`: BEV 동일 파일
- `/tmp/soa_no_accum_build.log`, `/tmp/soa_no_accum_tests.log`, `/tmp/soa_no_accum_plugin.log`

**판정:** 요청한 smoother 제한 해제와 SOA 시간 누적 제거는 구현·설치·회귀 테스트 완료. GT 재생에서는 정지 출력이 없었다. BEV의 PO 재최적화 실패 및 정지 출력은 미해결이며 전체 무정지 회피 성공으로 판정하지 않는다.
