# Localization agent context

이 디렉터리의 localization 작업 전 다음을 읽는다.

1. `../../../docs/localization/README.md`
2. `../../../docs/localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`
3. `../../../docs/localization/260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md`
4. 작업과 관련된 최신 EVIDENCE 또는 PLAN
5. 현재 launch/config default

`docs/localization/legacy/`는 명시적 재현 요청이 없으면 현재 구현 기준으로 사용하지
않는다. Production 변경 전 동일 raw bag offline A/B, `od_gt` 경량 폐루프, GPS 음영,
LAN-to-LAN gate를 순서대로 확인한다. Perception/planning/control 정책을 이 지침에서
추론하지 않는다.
