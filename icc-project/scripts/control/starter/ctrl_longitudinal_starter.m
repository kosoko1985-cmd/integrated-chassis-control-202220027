function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
    % 1. 상태 초기화
    if ~isfield(ctrlState, 'intErrorVx')
        ctrlState.intErrorVx = 0;
        ctrlState.prevForce = 0;
    end
    
    % 2. speed-tracking PI 제어
    vxError = vxRef - vx;
    
    % Anti-windup
    ctrlState.intErrorVx = ctrlState.intErrorVx + vxError * dt;
    ctrlState.intErrorVx = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intErrorVx));
    
    Fx_raw = CTRL.LON.Kp * vxError + CTRL.LON.Ki * ctrlState.intErrorVx;
    
    % 3. 저크 제한 (Jerk limit)
    m = 1700; % 기본 질량
    max_dF = LIM.MAX_JERK * m * dt;
    
    Fx_req = max(ctrlState.prevForce - max_dF, min(ctrlState.prevForce + max_dF, Fx_raw));
    ctrlState.prevForce = Fx_req;
    
    forceCmd.Fx_total = Fx_req;
    
    % 4. ABS modulation
    forceCmd.brakeRatio = ones(4, 1);
    kappa_target = 0.12;
    
    if ax < 0 && isfield(ctrlState, 'wheelSlip') 
        for i = 1:4
            if abs(ctrlState.wheelSlip(i)) > kappa_target
                forceCmd.brakeRatio(i) = 0.5; 
            end
        end
    end
end
