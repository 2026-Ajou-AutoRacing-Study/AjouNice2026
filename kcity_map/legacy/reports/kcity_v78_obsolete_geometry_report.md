# K-City v78 obsolete highway geometry cleanup

## 원인

v76에서 고주로 lanelet을 분할하고 공유 경계를 재구성할 때, 새 경계로
교체된 기존 way가 OSM XML에 독립 geometry로 남았다. v77은 활성 경계의
형상을 사각형으로 정리했지만 이 무참조 way를 제거하지 않았다.

`104165`는 v75까지 `201438.left`였으나 v76부터 어떤 relation에서도
참조되지 않았다. 임시 split way `105553`도 무참조 상태였으며
`104165`의 `131840 -> 45533 -> 45534` 구간과 중복됐다. 따라서 지도
편집기에는 실제 lanelet 경계와 별개인 이중 선이 표시됐다.

## v78 수정

- 교체된 기존 lanelet way 40개 제거
- v76 생성 과정에서 남은 무참조 임시 way 16개 제거
- 위 way와 v77 단순화 과정에서 고립된 좌표 전용 node 6,008개 제거
- `104165`, `105553`, `45533` 제거
- 실제 접속점 `131840`과 이를 사용하는 활성 경계 유지

## 검증

- lanelet 1,313개와 relation 1,390개 유지
- v77 대비 모든 relation 및 남은 활성 way/node 내용 동일
- relation이 참조하는 way 제거 0개, dangling node reference 0개
- 전체 routing graph 변화 0개
- 고주로 사각형 대상 39/39, invalid polygon 0개
- 물리적 평행 구간 양방향 LC 27쌍 유지
- malformed dashed boundary 0개
- 150 km/h lanelet 49개 모두 predecessor와 successor 유지
- 속도 규정 위반 0개; 보수 여유 시작 1.059 m, 종료 1.024 m

맵 SHA-256은
`57bcb7553d2aca1adfbd52d715f53bb767dadf85174947ec6283e242ff8845ab`이다.
