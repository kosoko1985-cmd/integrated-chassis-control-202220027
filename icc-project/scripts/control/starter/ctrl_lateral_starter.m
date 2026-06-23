function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
    % 1. 상태 초기화 (적분기)
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    
    % 2. yaw rate 추종 (AFS - PID 제어)
    yawError = yawRateRef - yawRate;
    
    % Anti-windup
    ctrlState.intError = ctrlState.intError + yawError * dt;
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));
    
    % Speed scheduling
    v_ref = 15; 
    vx_safe = max(vx, 1.0); 
    speed_factor = min(vx_safe / v_ref, 2); 
    
    steer_req = (CTRL.LAT.Kp * yawError + CTRL.LAT.Ki * ctrlState.intError) / speed_factor;
    deltaAdd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_req));

    % 3. slip angle 제한 (ESC)
    beta_th = deg2rad(3); 
    if isfield(LIM, 'MAX_SLIP_ANGLE')
        beta_th = LIM.MAX_SLIP_ANGLE;
    end
    
    if abs(slipAngle) > beta_th
        K_beta = CTRL.LAT.Kp; 
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * speed_factor;
    else
        deltaAdd.yawMoment = 0;
    end
end
