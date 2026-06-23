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
    
    % 상태 초기화 (적분기)
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    
    % (1) yaw rate 추종을 위한 AFS (PID)
    yawError = yawRateRef - yawRate;
    
    % Anti-windup을 적용한 적분항 업데이트
    ctrlState.intError = ctrlState.intError + yawError * dt;
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));
    
    % (3) Speed scheduling: 저속에서는 조향을 키우고 고속에서는 줄임
    v_ref = 15; % 기준 속도 [m/s]
    vx_safe = max(vx, 1.0); % 0 나누기 방지
    speed_factor = min(vx_safe / v_ref, 2); 
    
    % 기본 제어 입력 계산 (Speed scheduling 적용)
    steer_req = (CTRL.LAT.Kp * yawError + CTRL.LAT.Ki * ctrlState.intError) / speed_factor;
    
    % (4) limit/saturation (조향각 제한)
    deltaAdd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_req));

    % (2) slip angle 임계 초과 시 yaw moment 계산 (ESC)
    beta_th = deg2rad(3); % 임계값 (필요시 LIM.MAX_SLIP_ANGLE 로 교체)
    if isfield(LIM, 'MAX_SLIP_ANGLE')
        beta_th = LIM.MAX_SLIP_ANGLE;
    end
    
    if abs(slipAngle) > beta_th
        % driver intent와 반대 방향의 요 모멘트 생성
        % M_z = -K_beta * sign(beta) * (|beta| - beta_th) * f(vx)
        K_beta = CTRL.LAT.Kp; % 별도의 ESC 게인이 없다면 LAT.Kp 활용
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * speed_factor;
    else
        deltaAdd.yawMoment = 0;
    end
end

end
