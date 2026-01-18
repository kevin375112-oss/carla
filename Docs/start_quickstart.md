# 상대 차량의 끼어들기 의도 파악 및 대응 로직
def detect_and_handle_cut_in(side_sensors_data):
    # 1. 측면 센서(LiDAR + Camera)로부터 주변 차량 리스트 획득
    neighbor_vehicles = sensor_fusion.get_tracked_objects(side_sensors_data)
    
    for vehicle in neighbor_vehicles:
        # 2. 상대 차량의 횡방향 속도(Lateral Velocity) 계산
        lat_vel = vehicle.get_lateral_velocity()
        
        # 3. 차선 침범 확률 계산 (차선과의 거리 및 각도 고려)
        probability = prediction_model.calc_lane_change_prob(vehicle)
        
        if probability > 0.8 or lat_vel > threshold:
            # 4. 판단: 위험 상황으로 분류하고 TTC 계산
            ttc = calc_time_to_collision(self.pos, vehicle.pos)
            
            if ttc < emergency_threshold:
                # 5. 제어: 긴급 감속 명령 하달
                control.apply_brake(intensity='High')
                hmi.warn_driver("Emergency Braking: Cut-in detected")
