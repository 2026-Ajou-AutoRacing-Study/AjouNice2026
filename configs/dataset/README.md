# Base5 공통 보강 프로파일

현재 입력은 [`base5_midrange_reinforcement_v163.json`](base5_midrange_reinforcement_v163.json)이다.
환경별 생성기 사본이 아니라 공통 `scripts/portable_base5/reinforcement_profile.py`에 명시적으로 전달한다.
v163은 생성 불가 후보 근거를 반영한 149장 거리 재할당 프로파일이다.
파일 존재·CPU 생성 성공은 본수집 승인이 아니다.

실제 source·등록·native·release·완료 상태는 [데이터 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 따른다.
설계·실패·후속 마감 근거는 [Foggy13 기록](../../dataset/morai_od/docs/261019_base5_foggy13_reinforcement_and_capture.md)에 있다.
v161·v162는 [legacy 이력](legacy/README.md)으로 분리했다. 현재 runtime의 승인 입력으로 자동 재사용하지 않는다.
