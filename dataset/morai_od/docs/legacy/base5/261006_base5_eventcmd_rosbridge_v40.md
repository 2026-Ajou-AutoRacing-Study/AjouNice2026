# Base5 v40 — EventCmd 응답 혼선 재현·로컬 보완 / bridge 재연결 필요

2026-10-06. EVIDENCE. 본10K 캡처·학습 미시작. Production 스택 변경 없음.

## 결론

설치된 rosbridge의 **서비스 세대 간 요청 ID 재사용·공유 응답 저장소** 결함을
실제 설치 코드로 네트워크 없이 재현했다. 이전 handler가 새 handler의 응답을 가져갔다.
서비스 duplicate advertise 시 기존 response-wait loop를 종료하지 않는 경로도 확인했다.
live 로그에는 ScenarioLoad 중 해당 서비스 덮어쓰기 및 이전 등록이 남아 있는 시점의
요청 timeout이 관측됐다. 반복 Park 자체가 반드시 실패한다는 가설은12회 정지 실험에서
지지되지 않았다. rosbridge 세대 혼선은 주요 원인으로 판단하지만 **모든 live timeout이
이 결함 하나로 설명됐다는 의미는 아니다.** guarded bridge live 검증이 남는다.

## 근거

공통 출력 root: `/home/luv/morai_cap/base5_v2_261006/`.

- `park_preflight_v40.json`: 정지/Park 사전 정상,약5.4ms.
- `park_sequence_v40.json`: scene load/capture/Drive 없이 gear-only Park12회.
  요청 간격0/50/250ms 각각4회,전부 정상,응답약2.9–14.8ms.
- `batch_pending_v40/chunk_*/capture/event_cmd_requests.jsonl`: 실패 시 worker stack.
  Master조회/연결이 아니라 `transport.receive_once → sock.recv`에서응답대기했다.
  TCP request 송신 뒤 응답 미수신이며,MORAI가WS요청을실제로받았다는증거까지는아니다.
- 첫75셀 control 요청시작18:26:24.842→18:26:29.843timeout,
  rosbridge18:26:25.072 `Duplicate service advertised. Overwriting /Service_MoraiEventCmd`.
  이전 등록에 요청한 뒤 새로운서비스세대로교체되는race가존재한다.
- 99셀 finalPark와925셀 finalPark도timeout. 모든실패worker는새cleanup으로종료됐다.
  이후재시도는이전client thread와겹치지않았다.
- `rosbridge_service_generation_v40.json`: 실제설치된 `AdvertisedServiceHandler`에
  fake protocol만주입해기존/새handler를생성했다. 기존/새ID둘다
  `service_request:/probe:1`,응답dict동일,새세대응답을기존handler가소비하는현상재현.
  guard적용후dict분리·ID세대UUID분리·새응답오소비없음. 실제ROS/MORAI요청은보내지않았다.
- 기존bridge PID1368408이CPU약107%/20thread로관측됐다. 오래된busy-wait의영향은
  코드상가능하지만thread별CPU/stack을측정하지않아CPU원인을전부확정한것은아니다.

원본ROS로그는 `$HOME/.ros/log/latest/rosbridge_websocket-1.log`이며경로는다음실행에서
바뀔수있다. 위시각/문구를근거로사용한다.

## 코드 보완과 한계

1. `base5_event_cmd.py`: timeout시fresh persistent proxy를close하여TCP수신thread를
   깨우고정리한다. 호출마다새proxy를사용하고즉시닫으므로reload간연결재사용은없다.
   미종료worker가있으면추가요청을금지한다. phase/scene/latency/timeoutstack/정리결과를저장한다.
   timeout예산이나retry횟수/Park승인조건은늘리거나완화하지않았다.
   client연결종료는MORAI에이미전달된명령을취소하거나serverwaiter를제거하는것은아니다.
2. `base5_rosbridge_service_guard.py`: 응답dict를handler별로분리하고요청ID에세대UUID를
   추가한다. ID생성lock과duplicateadvertise시이전handler종료를추가했다.
3. `run_base5_rosbridge.py`: 설치된bridge진입점앞에서opt-in guard만적용한다.
   `/opt/ros` 파일/production alias를수정하지않는다.9090사용중이면실행을거부한다.
   기존실행중bridge에는소급적용되지않는다. MORAI가확장된opaqueID를정상echo하는지는
   재연결후live에서확인해야한다. 미검증상태에서본수집을시작하지않는다.

## 남은12셀 실행 결과

`batch_pending_v40`는기존20셀과중복없이추가8quota를통과했다.
75/76/87/99 및100/106/114/126은boundary/finalPark확인후선택했다.
첫chunk완료pause→동일계약resume도통과했다.
다음chunk의267/406/925는raw를촬영했다.925는`actor_annotation_above_ground` review이며
boundaryPark는통과했지만종료finallyPark2회timeout으로중단했다.
이chunk3raw는전달quota미편입이며1028은로드하지않았다.
안전미확인세션을재개/소급승인하지않는다.
따라서세pilot의최종선택은 **28/32셀**이고267/406/925/1028 네셀은새output재검증이필요하다.
Animal/Z보완포함전체32PASS를주장하지않는다.본10K미시작이다.
추가8장CPUinfo변환은train7/val1/test0·GT13개(vehicle4/pedestrian2/obstacle5/animal2)로
통과했다.세pilot선택합계28장/41GT다.925동물의높이review를이성공표본으로대체하지않는다.

## 사용자 재연결 순서

기존 `roslaunch rosbridge_server rosbridge_websocket.launch` 터미널만종료한다.
ROS master는그대로유지한다.기존9090이종료됐는지확인후아래를실행한다.

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

`Base5 rosbridge service guard: ...` 출력확인후MR에서기존7-interface Network를
Disconnect→Connect한다. 네트워크/sensor profile을새로변경하지않고CaptureMode를유지한다.
연결완료후사전Park/정지probe→guarded bridge 소량request→남은4셀을새output에서검증한다.
문제발생시stop하고원본보존한다.기존v39/v40실패contract를수정하지않는다.

guarded bridge 실행을테스트하면서기존bridge의9090점유거부가정상동작함을확인했다.
실제bridge재시작·MR재연결은아직수행하지않았다.
단위테스트는Base5총118개PASS.설치된ROS파일에직접patch하지않았고기존bridge도종료하지않았다.
