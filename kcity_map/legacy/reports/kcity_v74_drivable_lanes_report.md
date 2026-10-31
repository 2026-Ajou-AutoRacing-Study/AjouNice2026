# v74 지도와 competition route 정책

## 사용자 의도

제외는 competition route의 주행 가능 primitive 목록에 적용한다. 물리적인 도로 lanelet을 지도에서 삭제하지 않는다.

## 지도

- 파일: lanelet2_map_v74.osm
- SHA-256: ab1e62ab2f4429bbe81863de51db06eca1dea83c27a9491d27c46f3259c06995
- v73의 노드, way, relation ID 및 모든 lanelet 형상을 보존. lanelet 1,299개.
- 102241, 102237, 101992, 105462 경계를 dashed에서 solid로 복구. 경계를 사이에 둔 LC edge가 사라짐을 확인.
- 201456 → 201438 → 200128 합류 topology 유지.
- 200141은 존재하며, 기존 successor 단절은 남아 있다. 다음 구간 후보 201445와 오른쪽 접점이 약 44.86 m 떨어져 있다. 이번 route에서 제외하고 임의로 경계를 재작성하지 않았다.

## Route 정책

모든 segment에서 201400, 201440, 200148, 201445, 200141, 200186, 201437, 201463을 제외한다.
201435 segment에는 [201436, 201435, 200099, 201438]을 포함한다. 201438은 합류 접근 차선이며 옆 차선으로 LC 연결을 추가한 것은 아니다.

## 검증 및 적용

- v73 대비 relation/node 변경·삭제 없음. 변경된 way는 실선 복구 4개뿐.
- 잘못된 polygon, 누락 참조, 인접 중복 좌표 각각 0개.
- 남은 평행 차선 13쌍 양방향 LC 26개와 합류 chain 4개 검증.
- 실제 설치된 Mission Planner 및 실제 v74 map loader로 Route A 45 segments, Route B 55 segments 생성 성공. 제외 8개 불포함, 201438 포함, 기존 waypoint preferred 순서 일치.
- fallback 27개 singleton segment에서 제외 차선 없음.
- 소스 및 설치 공간 97 runtime version references가 v74이며 SHA 일치.
- 관련 런치/route requester 패키지 install_aw149 및 install_planning_addons 재빌드.

planning 재시작 후 Route A를 새로 요청한다. 별도 실행 중인 tracking/map-based prediction 및 지도 시각화는 재시작해야 새 지도 파일을 읽는다.

실제 route 검증 로그: /tmp/v74-route-policy-oracle.log
