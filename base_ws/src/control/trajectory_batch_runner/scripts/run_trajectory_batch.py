"""
목적:
    config에 지정된 trajectory 폴더의 모든 CSV를 MORAI에서 순서대로 검증한다.

역할:
    기존 udp, maptf_morai_cor, traj_gen, traj_val, traj_fol_a를 새로 구현하지 않고
    필요한 순서대로 실행하고 종료하는 조율기 역할만 담당한다.

대략적인 로직:
    1. Lane Change 또는 Obstacle Avoidance config를 읽는다.
    2. 지정된 trajectory 폴더의 CSV 목록을 파일명 순서로 만든다.
    3. 각 CSV 첫 행에서 시작 lanelet과 시작 속도를 구한다.
    4. 같은 lanelet과 속도를 가진 MORAI 시나리오를 선택하고 로드한다.
    5. traj_gen, traj_val, traj_fol_a를 순서대로 실행한다.
    6. traj_val 기록 완료 후 관련 프로세스를 종료하고 다음 CSV로 넘어간다.

현재 상태:
    실제 실행 코드는 아직 작성하지 않았다.
"""
