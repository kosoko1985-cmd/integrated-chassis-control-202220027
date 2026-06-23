function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    
   % 휠 반경 (힌트 참조)
    r_w = 0.33; 
    
    % (1) lonCmd.Fx_total -> 4-wheel 균등 brake (with 60:40 split)
    T_brake_base = zeros(4, 1); % [FL; FR; RL; RR]
    if lonCmd.Fx_total < 0
        F_brake_total = abs(lonCmd.Fx_total);
        
        % 전/후륜 60:40 분배 후 좌/우로 2등분 (T = F * r_w)
        T_brake_base(1) = (F_brake_total * 0.6 / 2) * r_w; % FL
        T_brake_base(2) = (F_brake_total * 0.6 / 2) * r_w; % FR
        T_brake_base(3) = (F_brake_total * 0.4 / 2) * r_w; % RL
        T_brake_base(4) = (F_brake_total * 0.4 / 2) * r_w; % RR
    end
    
    % (2) latCmd.yawMoment -> 4-wheel 차동 brake
    T_esc = zeros(4, 1);
    Mz = latCmd.yawMoment;
    
    if Mz ~= 0
        ratio_f = 0.6; % 전륜 개입 비율
        t_f_half = VEH.track_f / 2;
        t_r_half = VEH.track_r / 2;
        
        % 요구되는 좌우 힘의 차이 계산 (dT = F * r_w)
        dT_f_trq = (abs(Mz) * ratio_f / t_f_half) * r_w;
        dT_r_trq = (abs(Mz) * (1 - ratio_f) / t_r_half) * r_w;
        
        if Mz > 0
            % 양의 Mz (CCW 반시계 방향) -> 좌측 브레이크 증가
            T_esc(1) = dT_f_trq; % FL
            T_esc(3) = dT_r_trq; % RL
        else
            % 음의 Mz (CW 시계 방향) -> 우측 브레이크 증가
            T_esc(2) = dT_f_trq; % FR
            T_esc(4) = dT_r_trq; % RR
        end
    end
    
    % 총 제동 토크 합산 및 ABS brakeRatio 적용
    T_total = T_brake_base + T_esc;
    if isfield(lonCmd, 'brakeRatio')
        T_total = T_total .* lonCmd.brakeRatio;
    end
    
    % (5) 최종 saturation (0 ~ MAX_BRAKE_TRQ 제한)
    actuatorCmd.brakeTorque = max(0, min(LIM.MAX_BRAKE_TRQ, T_total));
    
    % (3) latCmd.steerAngle -> actuatorCmd.steerAngle (saturation)
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    
    % (4) verCmd -> actuatorCmd.dampingCoeff (pass-through)
    actuatorCmd.dampingCoeff = verCmd;

end
