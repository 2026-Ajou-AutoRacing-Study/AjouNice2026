# Planning C++ ABI 일치와 정적 장애물 크래시 방지

상태: **현재 PC clean 재빌드·격리 bag 재생 통과, MORAI 폐루프 재검증 전**
(2026-10-01).
이 문서는 재사용할 빌드·검증 계약만 다룬다. 개별 rosbag 및 core 분석은
Git 제외 경로 `planner_logs/analysis/260930_static_obstacle_abi/`에 둔다.

## 문제와 영향

`DrivableAreaInfo::Obstacle`의 C++ 정의가 변경됐는데, 현재 PC의 설치 라이브러리에는
옛 정의로 컴파일된 `scene.cpp.o`와 새 정의로 컴파일된 `utils.cpp.o`가 함께 링크됐다.
현재 헤더의 객체 크기는 144바이트이고, 설치본 `scene.cpp`가 벡터를 순회하는
간격은 112바이트다. `utils.cpp`가 만든 장애물 벡터를 `scene.cpp`가 잘못된
경계로 읽으면서 `StaticObstacleAvoidanceModule::plan()`에서 SIGSEGV가 난다.

이는 해당 시나리오의 차량 종류나 GNSS 음영 자체가 원인이 아니다. 정적 장애물
회피가 비어 있지 않은 `Obstacle` 벡터를 처리하는 다른 배치·경로에서도 재현될 수
있다. 반대로 장애물이 보이지 않거나 해당 회피 코드 경로를 타지 않으면 잠복한다.
`catkin_make_isolated`의 전체 패키지 성공 표시만으로 각 object file이 새 헤더에
맞춰 다시 컴파일됐다고 판단하지 않는다.

## 복구 및 합격 조건

1. MORAI, Planning, controller를 종료한다. 현재 PC의 소스 수정이나 사용자
   rosbag은 지우지 않는다.
2. `planning_ws/{build_isolated,devel_isolated,install_isolated}`의 **생성물만**
   제거하고 `planning_ws/tools/build.sh`로 planning 전체를 clean 빌드·설치한다.
   `scripts/workspace/build_all.sh --clean`은 base overlay만 정리하며 기존 planning
   install에 완료 marker가 있으면 planning 재빌드를 건너뛰므로, 이 문제의 단독
   해결 명령이 아니다. planning install을 지운 뒤 base overlay도 다시 빌드한다.
3. 재빌드 후 `scene.cpp.o`와 `utils.cpp.o`가 같은 새 빌드에서 생성됐는지,
   설치본 정적 회피 라이브러리가 144바이트 구조체에 맞는 순회 코드를 쓰는지
   확인한다. 빌드 로그의 `154/154`만으로 끝내지 않는다.
4. **같은 hidden S2 시나리오**에서 두 정적 차량이 등장하는 지점을 수동/자동으로
   지나며 BPP process 생존과 `/planning/trajectory`의 fresh 발행을 확인한다.
   Route `VERIFIED`나 시작 직후 몇 개 trajectory만으로 합격시키지 않는다.
5. GNSS 음영 진입·재획득까지 확인한 뒤 controller를 물린다. 별개의 trajectory
   품질/회피 실패는 ABI 크래시와 구분해 기록한다.

재발 방지: `data_types.hpp`처럼 여러 planning 패키지에서 공유하는 C++ 구조체의
필드·크기·레이아웃이 바뀌면, 증분 빌드를 신뢰하지 말고 planning 생성물 clean
빌드와 장애물 회피 runtime 회귀를 실시한다. CI에도 헤더 변경 시 역의존 패키지
재빌드 및 정적 장애물 1개 이상을 통과하는 BPP 검사를 추가하는 것이 적절하다.
현재 `depend.make`에는 `scene.cpp.o → install_isolated/.../data_types.hpp`
의존성이 **기록돼 있다**. 따라서 원인을 단순히 CMake 의존성 선언 누락으로
단정할 수 없다. 헤더 내용과 파일 timestamp가 어긋난 경로, install 재사용
방식까지 조사해야 하지만, 대회 전 운영 복구는 우선 clean 빌드로 고정한다.

팀장님 PC에서 문제가 없었던 이유는 **아직 확인되지 않았다**. 그 PC가 일관된
빌드 산출물을 가졌거나, 시험 중 같은 장애물 회피 분기를 밟지 않았을 수 있다.
이 둘은 팀장님 PC의 `scene.cpp.o`/설치 라이브러리와 당시 실행 로그를 비교해야
구분된다. 동일 소스·시나리오라는 사실만으로 설치 바이너리도 동일하다고
가정하지 않는다.

## 현재 PC 복구 확인 (2026-10-01)

Planning의 세 생성 디렉터리를 비우고 `scripts/workspace/build_all.sh --clean`으로
Planning 전체와 base overlay를 다시 빌드했다. 명령은 exit 0으로 완료됐다.
새 `scene.cpp.o`와 `utils.cpp.o`가 같은 빌드에서 생성됐고, 설치 `.so`의
해당 장애물 벡터 순회 간격은 이전 `0x70`이 아닌 `0x90`이다.
정적 회피 계산 테스트 24개, 플러그인 로딩 1개, ROS master를 붙인 관리자 초기화
1개를 통과했다.

기존 hidden S2 bag을 격리된 ROS master에서 v85 map loader와 함께 한 번
재생했을 때, 정적 장애물이 들어간 장면 76프레임(장애물 최대 1개)과 BPP 경로
156개를 관찰했고 BPP는 이전 크래시 시점을 넘어 생존했다. 이 재생에서는
bag 후반의 경로 발행이 멈췄으므로 **전체 주행 합격으로 판정하지 않는다**.
MORAI에서 같은 시나리오를 새 설치본으로 주행하고, 해당 지점 이후의 fresh
trajectory와 controller 동작을 확인해야 출전 전 gate를 닫을 수 있다.
