# Base5 latency 단계 분해 / 예상 배수와 실제 비용 차이

2026-10-06. RTX5060Ti16GB, MORAI OFF, batch1 AMP, 같은 H01 bag3개 sample,
6회워밍업+30회 측정. Production/runtime/학습 설정은 변경하지 않았다.
wide fresh architecture는 비용 대조이며 정확도 결과가 아니다.

## 결론

이전1.5–2배 추정은 범위/이미지 확대에 따른 거친 비용 추정이었다.
현재 Base5는 범위만 늘린 모델이 아니라 voxel·BEV metric step·depth discretization도 바꾼 후보다.
또 전체 처리의 모든 단계가 이미지/면적에 비례하지 않는다.
느려진 단계는 실제 존재하나 다른 단계의 비용 감소가 상쇄했다.

## 측정 방법

`profile_base5_stages.py`가 model/preprocessor의 실제 메서드를 instrument한다.
각 단계 CUDA event를 기록하되 단계마다 synchronize하지 않고 전체 forward 끝에서만 대기한다.
값은 **GPU stream의 critical-path span(host-induced idle 포함)**이며 kernel self-time/FLOP가 아니다.
CPU호출 시간도별도JSON에 보존하지만 비동기GPU queue의이전작업대기를포함하므로GPU표와합산하지 않는다.
view geometry/depth/pooling/downsample은 view total의하위항목이며중복합산금지다.
상위stage합계와전체wall차이는약0.5ms다. 측정오버헤드가있지만 baseline65.27ms는
이전uninstrumented65.29ms, highres89.45ms는이전89.56ms와가깝다.

## 단계별 평균 ms

| 단계 | E25 기존 | E25 highres | Wide lowres | Base5 wide highres |
|---|---:|---:|---:|---:|
| CPU압축해제·resize·crop·pack(모델처리밖) |6.48|7.07|5.73|6.22|
| GPU upload·정규화 |0.66|0.74|0.82|0.83|
| Camera Swin backbone |14.97|22.35|15.00|22.31|
| Camera neck |1.48|2.06|1.50|2.07|
| Camera→BEV 변환 전체 |27.51|43.43|25.52|39.22|
| LiDAR voxelization |1.93|1.95|2.06|2.09|
| LiDAR sparse encoder |12.60|12.71|11.31|11.56|
| BEV fusion |0.49|0.49|1.01|1.02|
| BEV backbone |1.27|1.27|2.28|2.31|
| BEV neck |0.18|0.18|0.34|0.34|
| Detection head·decode |3.64|3.72|5.61|5.56|
| その他wall差分 |0.53|0.55|0.52|0.51|
| **モデル処理全体(CPU画像処理除外)** |**65.27**|**89.45**|**65.96**|**87.80**|

## 1. 이미지 확대가 전체 latency를1.59배 만들지 않는 이유

256×704→320×896의픽셀수는1.591배다. camera backbone은1.492배,
camera→BEV는1.578배로실제증가했다. 두단계만의증가분은약23.29ms로
전체forward증가24.18ms의약96%를설명한다.
하지만이미지가커져도LiDAR voxelization/sparse encoder와BEV geometry·head는거의그대로다.
E25모델시간의camera backbone+neck+view transform비중은약67.4%였으며,
나머지약32.6%는화소수에같이비례하지않는다. 전체89.45/65.27=1.370배,
CPU까지포함한이전service는1.345배가되는것이측정상자연스럽다.

## 2. 범위를 넓혔는데 lowres 비용이 비슷한 이유

| 항목 | E25 | Base5 현재 후보 |
|---|---|---|
| 의미있는ROI 종/횡 |±30/±30m|±100/±30m|
| LiDAR XY voxel |0.075m|0.10m|
| Camera BEV XY step |0.30m|0.40m|
| 내부 voxel grid X×Y |800×800|2000×640(횡±32m padding)|
| 융합 feature Y×X |100×100|80×250|
| Camera depth |1–60m,0.5m간격,118 bins|1–100m,1m간격,99 bins|
| Head query 수 |200|200|
| 클래스 |3|4(fresh 비용probe)|

- 범위 면적배수만보면약3.33배이나격자를굵게하고padding까지반영한denseBEV cell수는2배다.
  실제fusion0.49→1.01ms, BEV backbone1.27→2.28ms, head3.64→5.61ms로느려졌다.
- 하지만camera frustum 계산량은거리상한자체보다depth bins×이미지feature pixels에영향받는다.
  거리상한은늘었지만bins는118→99로줄었다.
  같은이미지에서view geometry14.19→11.77ms,depth/image lift3.44→3.18ms,
  BEV pool7.94→7.46ms다. downsample은0.60→1.74ms로증가하지만
  view전체는27.51→25.52ms로약1.99ms감소했다.
- 같은cloud의범위필터통과점은늘었으나굵은voxel로점들이합쳐졌다.
  표본별occupied voxels는기존6493/7729/6266에서현재5984/7048/5969로줄었고,
  sparse encoder12.60→11.31ms로약1.29ms감소했다.
  Sparse 연산을전체빈grid면적에선형비례한다고보면안된다.
- 증가한fusion/BEVbackbone/neck/head 비용약3.65ms와
  감소한view/sparse encoder약3.28ms가상쇄된다.
  lowres총65.27→65.96ms로거의같아지는직접적인단계별근거다.

CPU이미지전처리의0.5–1ms차이는직렬실험의CPU/cache변동도포함하므로
ROI확대가이미지decode를빠르게했다고해석하지않는다.

## 3. 더 촘촘한 격자·depth 대조

Production 설정이아닌in-memory config의fresh비용대조다.
semantic ROI±100/±30m는유지하며기존0.075mvoxel/0.3mBEV를위한내부범위는
stride16에맞춰±100.8/±32.4m로padding했다. 이padding차이도기록한다.

| 비용대조 | 이미지 | voxel / BEV step | depth step | 모델전체평균 ms |
|---|---|---|---|---:|
| Wide lowres 현재격자 |256×704|0.10 /0.40m|1m|65.96|
| Wide lowres 기존격자 |256×704|0.075 /0.30m|1m|74.20|
| Wide highres 현재격자 |320×896|0.10 /0.40m|1m|87.80|
| 위와동일,depth만촘촘 |320×896|0.10 /0.40m|0.5m|119.89|
| Wide highres 기존격자+촘촘depth |320×896|0.075 /0.30m|0.5m|129.21|

depth만1→0.5m로바꾸면bin수99→198, 이미지·ROI·classes는동일하다.
위대조에서forward는87.80→119.89ms(+36.5%)다.
기존격자·0.5mdepth해상도로100m범위를고해상도에서보는마지막대조는
E25기존65.27ms의약1.98배다. **1.5–2배비용가능성은실제로존재하나현재후보는같은정밀도가아니다.**
current의0.10mvoxel/0.40mBEV/1mdepth가원거리·작은객체정확도를유지하는지는아직미학습이라모른다.
속도측정만으로이계약을최선이라고확정하지않는다.

## 4. ROS overhead와 GPU모델비용을 구분

앞선원속도E25 ROS측정의평균81.30ms(receipt→TCP응답)를분해하면:

- host전처리10.07ms(PointCloud2→XYZI약0.18ms포함).
- server입력전처리1.88ms.
- server model forward65.80ms.
- request69.43ms에서위server두단계를뺀transport/결과처리잔여약1.75ms.
- end-to-end에서host/request를뺀스케줄링·대기등잔여약1.80ms.

후자의두값은잔차추정이지별도network/queue profiler결과가아니다.
offline65.29ms와ROS65.80ms의model시간은가깝고,
ROS가모델자체를1.5–2배느리게하는증거는없다.
pipeline overlap과latest-only drop때문에throughput은1/end-to-end latency와같지않다.

## 원본 / 검증 / 후속

Git ignored `artifacts/evaluation/bevfusion/capacity_261006/`:

- `stage_profile_fixed.json`:5가지설정의모듈/substage/event·host timing·실제feature shape·voxel수.
- `stage_depth_control.json`:추가2가지depth/finegrid 대조.
- `stage_profile.json`:초기report에서constructor가config의voxelize_cfg를pop해보고실패한진단이력.
  이후modelinit에config deepcopy를사용해재측정했고최종7가지모두성공했다.

기존`benchmark_base5_capacity.py`와동일Docker/mount/PYTHONPATH환경에서:

```bash
python perception/detection3d/bevfusion/scripts/profile_base5_stages.py \
  --samples /path/to/samples --output /path/to/stages.json --iterations 30
```

추가depth대조는`--cases wide_highres_half_depth wide_dense_highres`다.
학습/Production/Planning/Control/Localization은변경하지않았고새데이터도수집하지않았다.
다음은Base5학습후원거리성능과이격자/depth계약의정확도비용을검증하는것이다.
추후순수최적화후보는고정calibration의view geometry 캐시지만이번에는구현하지않았다.
