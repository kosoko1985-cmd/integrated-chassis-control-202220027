function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
    dampingCmd = zeros(4, 1);
    
    for i = 1:4
        zs_dot = suspState.zs_dot(i);
        zu_dot = suspState.zu_dot(i);
        v_rel = zs_dot - zu_dot;
        
        % Skyhook 제어
        if (zs_dot * v_rel) > 0
            c_req = CTRL.VER.skyGain * abs(zs_dot / (v_rel + 1e-6));
        else
            c_req = CTRL.VER.cMin;
        end
        
        % 감쇠력 제한 적용
        dampingCmd(i) = max(CTRL.VER.cMin, min(CTRL.VER.cMax, c_req));
    end
end
