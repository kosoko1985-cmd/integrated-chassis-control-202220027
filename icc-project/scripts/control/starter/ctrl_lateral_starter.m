function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
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
