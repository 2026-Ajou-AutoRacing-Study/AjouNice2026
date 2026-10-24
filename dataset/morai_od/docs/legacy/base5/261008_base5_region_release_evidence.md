# Base5 구역별 배치·같은5K의6환경 — v123 EVIDENCE

2026-10-08. Base3/E25 보강이 아닌 Base5-only from-scratch single-frame OD 준비다.
본5K/30K 수집과 모델 학습은 실행하지 않는다. Production/planning/control/localization은 변경하지 않았다.

## 최종 release와 구성

Root: `/home/luv/morai_cap/base5_v2_261008/regions_5k_v123_release`.
`scenarios_5k/manifest.json`은5000개고유payload 전수 OFFLINE_PASS/오류0,
`ordered/manifest.json`은정적4822→이동178순서이며 geometry와quota는같다.
최종 실행입력은 `environment_suite/sunny13/manifest.json`이다.6환경suite합계30000관측이다.

| 카테고리 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 고주로 |740|600|1340|
| 로터리 |0|350|350|
| 기타 차량 |80|50|130|
| 장애물 |770|600|1370|
| 사람·동물·복합 |830|250|1080|
| 독립 음영 |—|—|500|
| 빈 배경 |230|0|230|
| 전체 | | |5000|

음영500은별도family이며code의mission=false는기존motion adapter호환용이다.
고주로범용/미션장수에다시더하거나분포를겹쳐세지않는다.
positive4750/empty250(일반230+음영20), train4255/val745/test0이다.
6환경도같은group/split이므로전부승인된다면train25530/val4470이다. 이것은독립30K장면이아니다.

## 구현과 반복 개선

- `base5_region_contract.py`: family뿐아니라actual Ego/actor XY를MGeo corridor proxy로검사한다.
  고주로/로터리에는primary·secondary모두일반vehicle43종만둔다.
  다른비차량범용Ego는이두구역과음영밖이며, 비차량actor도차량구역밖이다.
  shadow는전체122종허용예외다. 이규칙을inference class mask/GT제거로쓰지않는다.
  corridor half-width4.5m는지도기반proxy이지공식법적영역/physics mesh인증이아니다.
- `revise_base5_region_release.py`: v122에서fresh release를만든다.
  범용2420+로터리350=2770geometry재생성, 나머지2230geometry보존이다.
  모든500shadow의scenario SHA는v122와같다. 비로터리미션/초근접/이동경로계약을유지했다.
- 로터리generic100을mission에흡수한다. 접근70(approaching30/waiting40)/entry80/
  circulation130/exit70. stale approaching 상태가원형구간에남는실제생성오류를수정했다.
  기존generic budget도제거하고mission상황별개수/밀집도로재배정했다.
- 단위자릿수0으로맞추기위해희귀/근접이아닌일반4셀을재배정했다.
  제거된고주로비차량혼합노출은허용구역의사람혼합270셀에서secondary실체장애물1개조건으로보충한다.
  581혼합사람셀전체를강제했을때사람노출감소가너무커270으로줄였다.
  장애물노출은v122의2971→2973으로유지, 일반사람2474→2162(-12.6%)이다.
  사람830범용의주객체·희귀quota와앞뒤좌우분포는유지하며감소는주로secondary다.
- 범용크기예산mode단일726/일반1210/밀집484=30/50/20%(2420셀), max8유지다.
  사람830의front415/left166/right166/rear83=50/20/20/10이다.
  미션은이비율을강제하지않고상황별장면을유지한다.
- 차량geometric center 거리quota와rear-axle placement차이를반영했다.
  Ego기준12m버스의전방6–10mcenter ring은충돌없이못채워두ring을같은family/front의작은차량과교환했다.
  swaps `[326,94]`, `[989,105]`로자산총량·거리총량을모두유지한다.
- ordinary adjacent차량은실제평행route에배치한다. Ego쪽topology도관측대상의반대lane로선택한다.
  정지관측을차선변경명령과혼동해can_move_lane=false만으로인접차로를없애지않는다.
  NPC는실제도로route의XY/yaw를사용하고임의횡jitter를넣지않는다.
- 첫33셀pilot에서32승인/1부족분(273.818s)이었다.
  `b5v2_00290` primary XC90 43.65m 앞에 DEFAULT_MPV12.56m와Kona31.60m가놓여
  가까운차량만166points, 나머지는0point/camera근거없음이었다. 재배치도같은방향차량에가려졌다.
  후방97.66m버스도앞쪽29.11m믹서에가려져최초review후bounded재배치로승인됐다.
  가까운dense장면의Woman5 runtime누락review도보존하고같은quota로대체했다.
- 이를재시도만으로덮지않고초기generic highway에완전가림footprint angular proxy를추가했다.
  보조box가주box보다완전히가깝고수평각95%이상을덮으면초기재생성한다.
  최종5K생성에서1179개불량candidate를이이유로거절했다. 부분가림은유지한다.
  높이가다른3D가시성/여러blocker합성가림은proxy로확정하지않고nativepositive검사를그대로유지한다.
  `regions_5k_v123_ready`의첫pilot/부족분원본은보존했다. 그output을새runtime으로resume하지않는다.

## 객체군별 계획 노출 — v123 동결 스냅샷

- 기록 시각: **2026-10-08 11:05:24 KST (UTC+09:00)**.
- 기준 버전: **Base5 v123 / `regions_5k_v123_release`**, 구현·검증 커밋 `c5ff1baf`.
- 계산 입력: `/home/luv/morai_cap/base5_v2_261008/regions_5k_v123_release/scenarios_5k/manifest.json`.
- 입력 SHA-256: `9e1e381633a7e28085a73bf6cae7b5fcb5ed439f3dce623c879d9267bc2c51e3`.
- 계산 근거: 같은 release의 `planned_exposure.json`이며 입력 hash 일치를 다시 확인했다.
- 5K는 이 manifest의 부모 객체 배치 노출, 30K는 **같은 구성의 6환경을 모두 취득한다는 계획의 6배**다.
  첫 Sunny13 5K + sunny11/15·foggy11/13/15의 추가25K이며 실제 취득 완료 수치가 아니다.

**이 표는 위 시각·버전·manifest에만 해당한다.** 나중에 객체군·배치 수·quota·manifest가 바뀌면
현재값으로 그대로 인용하지 않는다. 이 스냅샷은 보존하고 새 시각/버전/hash와 재계산 표를 별도 추가한 뒤
CURRENT의 최신 표 참조를 갱신한다. 재배치·부족분 보충 후 실제 가시 GT 수는 별도 ledger 집계로 기록한다.

아래는부모asset배치횟수다. 독립장면수/최종visible GT box수와같지않다.
한장면에같은군의객체가여러개있고복합객체는여러nativecomponent를만든다.
모든노출/자산별상세내역은release의`planned_exposure.json`에있다.

| 객체군 | 5K 계획 노출 | 30K 계획 노출 |
|---|---:|---:|
| 일반 차량 |4,115|24,690|
| 실체 장애물 |2,973|17,838|
| 일반 사람 |2,162|12,972|
| 인체 더미 |148|888|
| 동물 |143|858|
| 반려동물 동반 사람 |142|852|
| 유모차 |131|786|
| 자전거·스쿠터 등 탑승자 |148|888|
| 차량형 Obstacle 자산 |117|702|
| **합계** |**10,079**|**60,474**|

모든122종의planned-primary최소5회다. 이것이모든종의학습충분성이나최종visible최소5를보장하지않는다.
최종실제가시primary/거리/방향coverage를ledger로감사하고부족분은같은quota로보충한다.
최종개수분포0/1/2/3/4/5/6/7/8개=250/1966/1309/933/208/199/98/15/22셀이다.
1개장면에는원래단일범용뿐아니라미션단일상황도포함돼있다.

## 6환경과 GT/학습 계약

- sunny/foggy×11/13/15의각5Kmanifest. 첫sunny13뒤다른5환경25K다.
  weather/hour는operator UI에서적용·확인한다. manifest는환경을자동변경하지않는다.
- 정적4822/이동178(사거리154+음영24), 두waypoint/ground후보와Park/CollisionData계약유지.
  rigid정적자산을움직이지않으며열린G80은공식제외, 나무/시설물은기존배경이다.
- raw Instance GT/4class(vehicle/pedestrian/obstacle/animal)/7D/velocity off,
  LiDAR≥1point OR 실제camera visibility≥16px·최소extent4px(320×896crop),
  ROI[-100,-30,-3,100,30,2]계약유지다. 안보이는primary를빈negative로변환하지않는다.
- `prepare_base5_environment_suite.py`가동일entries/order/group/split을검사한다.
  변환시`--environment-tag`로`sunny13:source-cell`같은token을만들고group은고정한다.
  `merge_base5_environment_infos.py`는중복token/환경간split누수/서로다른GT계약을거부한다.
  camera/LiDAR/raw/native GT는변경하지않는다. merge는신뢰하는로컬PKL만사용한다.
- Dashboard는기존읽기전용구현을그대로연결했다. current child/recovery/approved/deficit/rate/
  ETA/free disk를표시하고새Sunny13output은`not_started`로확인했다.
  시작이후runtime/config/manifest/source를동결하며구output에새코드를섞지않는다.

## 최종 실제검증/소요시간

최종프리등록33셀은로터리phase×밀집12, 고주로앞뒤×근/중/원6, 범용3군단일/밀집6,
근접1·사거리걷기/달리기2·음영먼접근1·배경1·희귀4다.
최종33/33승인·부족분0·220.438초(약3분40초)다. 실제34capture의review원본도보존했다.
새목록33셀의최초32승인/품질review1셀은bounded same-quota대체로보충했다.
review는근접dense셀`b5v2_00082`의Woman5 secondary runtime/annotation누락이다.
주Concrete_Barricade01은LiDAR0point지만실제camera근거로positive였고동일quota로전체셀을대체했다.
이는33번모두처음부터성공했다는뜻이아니다. 통신halt/강제resume는없었다.
vehicle parent42/42·실체obstacle14/14·일반사람3/3·반려동물동반1/1·유모차2/2·탑승자1/1이positive다.
animal parent는4/7이positive였다. 안보이는secondary3개를GT에추가하지않고raw에보존했다.
33프레임의native GT72개(vehicle43/obstacle16/pedestrian8/animal5), 빈배경1개,
camera-only근접obstacle1개를CPU 변환에유지했다. images/4feature points/calibration/7D 검사오류0이다.
전체10child/recovery session종료Park/boundaryPark확인, unsafe0이다.
이동2개는사거리walking/running이다. 모든178mover나전체122종의신규live전수인증은아니다.
완료resume PASS:1182raw파일SHA/inode/mtime/size와10event로그·선택ledger·누적시간불변,
추가capture0이다. 355개Base5단위/회귀tests(74.374s) PASS, whitespace diff 검사PASS다.

이목록은일부러어려운사례를고른것이며5K성공률이나통계적신뢰구간을보장하지않는다.
음영독립terrain24셀은이pilot의이동2셀보다비싸고과거running1ground부족분은여전히가능하다.
통신/Park불확실은halt, 안전한품질실패는bounded대체/부족분으로기록하는기존정책을유지한다.

### 취득 시간·공간 추정

- 실측cost report:정적31셀6.619초/승인, 사거리이동2셀7.626초/승인이다.
  shared-batch child시간과부모overhead를안분한값이며재시도/종료비용이포함돼있다.
- 정적4822/이동178로가중하면5K **9.243시간**이다. 음영24개의독립terrain준비를
  사거리이동2개비용으로환산할수없고dense/asset/거리별완전가중도아니다.
  이전음영준비비용+품질재시도/장시간MR부하를포함한운영예산은 **10–13시간/5K**를유지한다.
  본수집첫100–200셀의누적rate로다시갱신한다. guaranteed ETA가아니다.
- Sunny13 5K+다른환경25K=30K는실측단순6배55.46시간, 운영예산60–78시간이다.
  환경설정/수동중단/모델학습·후처리시간은별도다. Foggy도같은가시성성공률이라고보장하지않는다.
- 현재pilot의unique-inode raw/QA/rejected비용외삽은5K당26.51GiB,6환경약159.05GiB다.
  당시여유약248GiB에서raw만은예상범위에있지만변환캐시/압축복제/실패분으로추가증가한다.
  환경마다디스크를확인하고사용자원본을자동삭제하지않는다.
- 대시보드는이번root의승인rate/부족분/실시간child/free disk/누적ETA를사용한다.
  MR에는5000파일을복사하지않고현재chunk의임시scenario/sidecar만설치·회수한다.

### 산출물

- `offline_audit.json`, `planned_exposure.json`, `revision_report.json`, `frozen_specs.json`
- `pilot_manifest.json`, `live_pilot`, `pilot_infos`, `readiness.json`, `cost_report.json`
- `completed_resume_verification.json`, `environment_suite/suite.json`

현재명령은[런북](261007_base5_5k_capture_runbook.md)을따른다. 본수집승인은사용자보고후다.
