# 인지 설정 작업 지침

[BEVFusion CURRENT](../../docs/bevfusion/CURRENT.md#runtime)와 변경할 config의 실제 사용 launch를 확인한다.
신호등 설정은 [해당 패키지](../../base_ws/src/perception/traffic_light_detection/README.md)를 따른다.

파일명이 최신이라는 이유로 production 기본값을 교체하지 않는다.
코드 기본값, opt-in 후보, 동결 실행의 resolved config를 구분한다.
문서와 실제 실행이 다르면 차이를 보고하고 해당 계약의 근거를 확인한다.
변경 후 CURRENT의 기존 항목과 관련 검사를 갱신한다. 계획 이력을 이 파일에 쌓지 않는다.
planning/control config는 이 지침의 범위가 아니다.
