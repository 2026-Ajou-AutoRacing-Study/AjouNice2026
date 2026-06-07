# fast lio - ros bridge
### fast-lio - ros bridge 만들기

### fast-lio 결과와 ego_topic GT 비교하기
- 이건 무조건 rosbag 하나 잡고 그거 기준으로 가야 함 
- 기준 frame이 다르니까 static TF 임시로 만들어서 비교?? -> 흠 고민
- 아님 차라리 두 odom의 첫 pose를 각각 0으로 보고, 이후 이동량만 비교하는 게 좋지 않을까
- yaw까지 정확히 비교하려면 heading도 0이나 그런 값으로 보정해야하는데 이럼 너무 복잡해질 것 같음 -> 처음은 그냥 pose만 0 초기화하고 x, y 이동량(+궤적)만 보자