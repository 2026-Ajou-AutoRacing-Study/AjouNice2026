# Base5 공통 보강 프로파일

현재 후보는 `base5_midrange_reinforcement_v163.json`이다. 환경별 생성기 사본이 아니라
공통 `scripts/portable_base5/reinforcement_profile.py`에 명시적으로 전달하는 입력이다.
프로파일 파일이나 CPU 생성 성공만으로 본수집이 승인되는 것은 아니다.

- v161: 최초 후보 이력. 당시 구현 snapshot과 실제 중단 근거를 보존한다.
- v162: CPU 생성 가능성 검사와 누적 근거리 보호 예산을 명시한 중간 후보다.
- v163: v162의 생성 불가 후보 근거를 반영한 현재 149장 거리 재할당 후보다.

정확한 실행 상태와 source·등록·native·release는
[데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 따른다.
설계 근거와 과거 실패를 포함한 검증은
[Foggy13 보강 기록](../../dataset/morai_od/docs/261019_base5_foggy13_reinforcement_and_capture.md)에 있다.
과거 JSON을 현재 runtime의 승인 설정으로 재사용하지 않는다.
