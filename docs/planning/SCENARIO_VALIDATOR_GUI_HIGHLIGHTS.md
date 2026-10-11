# 시나리오 검증기 — 이전 화면 참고 (2026-10-03)

**2026-10-11 간소화 이후 현재 버튼 안내는 [작업 순서](SCENARIO_VALIDATOR_WORKFLOW.md)를 우선하세요.** 아래 그림은 이전 편집형 GUI입니다. 현재는 일반 설정이 회색 읽기 전용이고 VSCode에서 YAML을 수정합니다. 경로 붙여넣기, planning 빠른 테스트, 설정 확인 탭이 추가됐고 세부 배치 버튼은 목록 관리로 모았습니다. 그림의 ‘개인 YAML 저장’, 신호등 환경 탭, 기본 탭 장면 선택은 현재 작업법이 아닙니다.

[설치 후 작업 순서](https://github.com/2026-Ajou-AutoRacing-Study/AjouNice2026/blob/planning/autoware-universe-to-ros1/docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md)를 따라 하면서 버튼 위치가 헷갈릴 때 봅니다. 모든 GUI 기능을 설명하려는 문서는 아닙니다.

아래 5장은 팀에서 제공한 2026-10-03 캡처를 바탕으로 **빨간 박스·설명을 붙인 안내 이미지**입니다. 편집 과정에서 글꼴·배치가 실제 화면과 미세하게 다를 수 있으므로 설정값·상태는 실제 GUI와 YAML을 우선합니다. 빨간 글씨는 권장 사용법이며, 캡처에 보이는 체크 상태가 권장 기본값을 뜻하지는 않습니다. 화면의 /home/luv 경로는 캡처 PC의 예시이고 본인 PC에서는 본인 홈 경로를 사용합니다.

현재 GUI에는 내 YAML 열기 옆에 **내 작업 폴더 열기** 버튼도 추가되어 있습니다. 원본 캡처에는 없어서 이미지에 새 버튼을 만들어 넣지는 않았습니다. 실제 파일 삭제·이름 변경은 그 버튼으로 파일 탐색기를 열어 진행합니다. 입력 사본 없는 이전 실험에는 재개 제약이 있으므로 현재 작업법은 위 링크를 우선합니다.

이전 버전에는 기본 탭의 파일 패턴 옆에 **장면 선택** 버튼도 있었습니다. 이름·패턴 검색과 검색 결과 전체 체크로 일부 JSON만 실행할 수 있습니다. 선택 목록 모드에서는 파일 패턴 칸이 비활성화되고 버튼에 선택 개수가 표시됩니다. 전체 실행(패턴)으로 돌아가면 파일 패턴을 다시 편집할 수 있습니다. 아래 캡처에는 이 버튼도 없으므로 실제 작업 순서를 우선하세요.

## 1. 기본 설정

처음에는 예제를 선택하고 **베이스 → 내 작업 폴더 복사**부터 누릅니다. 작업 복사본에서 이름·시나리오 폴더·파일 패턴을 확인하고, 목적지는 기본값을 유지합니다. 녹화는 영상 ON / 센서 bag OFF를 권장합니다.

![기본 설정에서 자주 쓰는 항목](https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/refs/heads/planning/autoware-universe-to-ros1/docs/planning/assets/scenario_validator_gui/01_basic_annotated.png)

## 2. 고급 / 네트워크

보통 토픽은 basic, 화면 배치는 split, 바 높이는 auto 그대로 씁니다. Network 경로는 MORAI에 실제로 로드한 파일과 맞아야 합니다. 환경 검사와 최초 실제 GPU 검사, 연결 JSON 경로 복사 위치를 표시했습니다.

![고급과 네트워크 탭에서 자주 쓰는 항목](https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/refs/heads/planning/autoware-universe-to-ros1/docs/planning/assets/scenario_validator_gui/02_network_annotated.png)

## 3. 신호등 환경

기본 탭에서 신호등 ON을 선택한 경우에만 확인합니다. 공용 가상환경을 이미 설치했다면 가상환경 Python을 고르고 기존 경로를 사용합니다. 시스템 Python을 사용하는 팀원은 기존 환경을 유지해도 됩니다. 신호등 OFF면 환경을 설치하지 않아도 됩니다.

![신호등 환경 탭에서 자주 쓰는 항목](https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/refs/heads/planning/autoware-universe-to-ros1/docs/planning/assets/scenario_validator_gui/03_traffic_light_annotated.png)

## 4. 실행 목록 / 배치

개인 YAML들을 추가하고 순서를 정해 전체 검사 후 새 배치 시작을 누릅니다. 중단한 배치를 이어갈 때만 배치 재개입니다. 목록 초기화는 YAML·녹화 결과를 삭제하지 않습니다.

![배치 탭에서 자주 쓰는 항목](https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/refs/heads/planning/autoware-universe-to-ros1/docs/planning/assets/scenario_validator_gui/04_batch_annotated.png)

## 5. 실험 기록 / 재개

시간·진행·P/F/E를 보고 이어갈 기록을 선택합니다. 선택 실험 재개는 **현재 편집 화면이 아니라 당시 보관된 입력**을 사용합니다. 같은 조건을 처음부터 다시 시험할 때는 상단 새 실험 시작을 사용합니다.

![실험 기록과 재개 탭에서 자주 쓰는 항목](https://raw.githubusercontent.com/2026-Ajou-AutoRacing-Study/AjouNice2026/refs/heads/planning/autoware-universe-to-ros1/docs/planning/assets/scenario_validator_gui/05_history_annotated.png)
