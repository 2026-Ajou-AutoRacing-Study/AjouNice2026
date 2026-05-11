# Roadmap

## Repository setup
- [x] `README.md`, `CONTRIBUTING.md`, `ROADMAP.md` 작성
- [x] `docs/`, `configs/`, `scripts/`, `maps/` 생성 후 개별 `README.md`, 예시 추가
- [x] 껍데기 workspace 생성 
- [x] 개별 스택들 `third_party/`에 cloning해오기
- [x] ~~`PATCH_NOTES.md` 생성 후 nested git 제거 (`find third_party -name .git -type d`)~~
  - [x] local cloning 방식으로 변경

## ROS
- [ ] 개별 스택들 ros workspace에 통합하기
  - [ ] topic interface 정의
  - [ ] 사용할 msgs 정의
  - [ ] msgs converting interface 정의? -> 고민

## MORAI

## Local setup(jsw)
- [ ] perception & localization 테스트 방법 고안 -> morai를 개인 pc에 wsl2로 설치? -> 시도 후 안되면 듀얼부팅
- [ ] 개인 PC에 ubuntu 20.04, ros1, morai 설치 -> 실행되나 확인 후 workspace 구성, morai bridge 작성, 등...