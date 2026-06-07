# morai - ros bridge 만들기
## 계획
- 본격적으로 bridge 제작 착수해서, 우리 스택들이 morai에 종속되지 않게 하고
- 판단 단에서 우리 내부 topic명만 보고 개발 가능하게 만들기
```
입력:
  /Ego_topic
  morai_msgs/EgoVehicleStatus

출력:
  /localization/odom
  nav_msgs/Odometry
```


### config 만들기
- fast lio 안에 `config/velodyne.yaml`가 있긴 한데 서브모듈로 관리할 거니까 우리 쪽에 config 따로 만들기
```bash
mkdir -p ~/AjouNice2026/configs/localization
gedit ~/AjouNice2026/configs/localization/fast_lio_morai_velodyne.yaml
```


### launch wrapper 만들기
- config 우리쪽 꺼 쓰기로 했으니까 그거에 맞춰서 launch도 새로 만들어줘야 함