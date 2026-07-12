#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from std_msgs.msg import Float32

class CutInPredictor:
    def __init__(self):
        rospy.init_node('cut_in_predictor', anonymous=True)
        
        # 1. Subscribers: 자차 상태와 주변 객체 상태 데이터 수신
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/Object_topic', ObjectStatusList, self.object_callback)
        
        # 2. Publishers: 가장 위험한 끼어들기 예상 시간(TTLC) 발행
        self.ttlc_pub = rospy.Publisher('/Prediction/Lowest_TTLC', Float32, queue_size=10)
        
        self.ego_status = None
        self.LANE_WIDTH = 3.0 # [m] 기준 차선 폭

    def ego_callback(self, msg):
        self.ego_status = msg

    def object_callback(self, msg):
        # 자차 데이터가 수신되지 않았다면 연산 대기
        if self.ego_status is None:
            return

        lowest_ttlc = float('inf') # 초기화: 무한대 [s]

        # 글로벌 좌표계에서의 자차 정보
        ego_x = self.ego_status.position.x # [m]
        ego_y = self.ego_status.position.y # [m]
        ego_heading = math.radians(self.ego_status.heading) # [rad]

        # 주변 NPC 차량 반복 연산
        for npc in msg.npc_list:
            # 1단계: 내 차량 기준의 로컬 좌표계(Local Coordinate)로 변환
            dx = npc.position.x - ego_x # [m]
            dy = npc.position.y - ego_y # [m]

            # 2D 회전 변환 적용
            local_x = dx * math.cos(ego_heading) + dy * math.sin(ego_heading) # 전방 거리 [m]
            local_y = -dx * math.sin(ego_heading) + dy * math.cos(ego_heading) # 횡방향(좌우) 거리 [m]

            # 필터링 1: 내 차량보다 뒤에 있거나 너무 멀리 있는(50m 초과) 객체 제외
            if local_x < 0 or local_x > 50.0:
                continue

            # 필터링 2: 인접 차선에 있는 객체만 관심 영역(ROI)으로 설정 (약 1.5m ~ 4.5m)
            if abs(local_y) < (self.LANE_WIDTH / 2.0) or abs(local_y) > (self.LANE_WIDTH * 1.5):
                continue

            # 2단계: 상대 기구학적 데이터 계산
            npc_heading = math.radians(npc.heading) # [rad]
            npc_velocity = math.sqrt(npc.velocity.x**2 + npc.velocity.y**2) / 3.6 # [km/h] -> [m/s] 변환
            
            # 요각 차이 계산
            delta_heading = npc_heading - ego_heading # [rad]
            
            # 내 차량을 향하는 횡방향 속도 성분 계산
            lateral_velocity = npc_velocity * math.sin(delta_heading) # [m/s]

            # 3단계: 끼어들기 의도 판단 (횡방향 속도가 내 차선 쪽으로 0.2 [m/s] 이상일 때)
            is_cutting_in = (local_y > 0 and lateral_velocity < -0.2) or (local_y < 0 and lateral_velocity > 0.2)

            if is_cutting_in:
                # 차선 경계선까지 남은 횡방향 거리 계산
                distance_to_lane_boundary = abs(local_y) - (self.LANE_WIDTH / 2.0) # [m]
                
                # 4단계: TTLC (Time To Lane Crossing) 계산
                if abs(lateral_velocity) > 0.01: # 0으로 나누기 에러 방지
                    ttlc = distance_to_lane_boundary / abs(lateral_velocity) # [s]
                    
                    # 가장 짧은 시간(가장 위험한 상황) 업데이트
                    if ttlc < lowest_ttlc and ttlc > 0:
                        lowest_ttlc = ttlc

        # 유효한 끼어들기 상황이 감지되었다면 토픽으로 발행
        if lowest_ttlc != float('inf'):
            self.ttlc_pub.publish(lowest_ttlc)
            rospy.loginfo(f"Cut-in Detected! TTLC: {lowest_ttlc:.2f} [s]")

if __name__ == '__main__':
    try:
        predictor = CutInPredictor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass