function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
    r_w = 0.33; 
    
    % 1. 종방향 4-wheel 균등 brake (60:40 분배)
    T_brake_base = zeros(4, 1); 
    if lonCmd.Fx_total < 0
        F_brake_total = abs(lonCmd.Fx_total);
        T_brake_base(1) = (F_brake_total * 0.6 / 2) * r_w; % FL
        T_brake_base(2) = (F_brake_total * 0.6 / 2) * r_w; % FR
        T_brake_base(3) = (F_brake_total * 0.4 / 2) * r_w; % RL
        T_brake_base(4) = (F_brake_total * 0.4 / 2) * r_w; % RR
    end
    
    % 2. 횡방향 (ESC) 차동 brake
    T_esc = zeros(4, 1);
    Mz = latCmd.yawMoment;
    
    if Mz ~= 0
        ratio_f = 0.6; 
        
        % 트랙 반거리 설정 (안전 처리)
        if isfield(VEH, 'track_f')
            t_f_half = VEH.track_f / 2;
            t_r_half = VEH.track_r / 2;
        else
            t_f_half = 0.78;
            t_r_half = 0.78;
        end
        
        dT_f_trq = (abs(Mz) * ratio_f / t_f_half) * r_w;
        dT_r_trq = (abs(Mz) * (1 - ratio_f) / t_r_half) * r_w;
        
        if Mz > 0 % CCW 방향
            T_esc(1) = dT_f_trq; % FL
            T_esc(3) = dT_r_trq; % RL
        else % CW 방향
            T_esc(2) = dT_f_trq; % FR
            T_esc(4) = dT_r_trq; % RR
        end
    end
    
    % 제동 토크 합산 및 ABS 반영
    T_total = T_brake_base + T_esc;
    if isfield(lonCmd, 'brakeRatio')
        T_total = T_total .* lonCmd.brakeRatio;
    end
    
    % 3. 최종 Saturation 처리
    actuatorCmd.brakeTorque = max(0, min(LIM.MAX_BRAKE_TRQ, T_total));
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    actuatorCmd.dampingCoeff = verCmd;
end
