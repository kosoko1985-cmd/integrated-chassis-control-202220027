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
   % 상태 초기화
    if ~isfield(ctrlState, 'intErrorVx')
        ctrlState.intErrorVx = 0;
        ctrlState.prevForce = 0;
    end
    
    % (1) speed-tracking PI
    vxError = vxRef - vx;
    
    % (4) anti-windup
    ctrlState.intErrorVx = ctrlState.intErrorVx + vxError * dt;
    ctrlState.intErrorVx = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intErrorVx));
    
    Fx_raw = CTRL.LON.Kp * vxError + CTRL.LON.Ki * ctrlState.intErrorVx;
    
    % (3) jerk limit (차량 질량을 곱해 힘의 변화율 한계 도출)
    m = 1700; % 기본 질량 (sim_params.m의 VEH.mass 활용 권장)
    max_dF = LIM.MAX_JERK * m * dt;
    
    Fx_req = max(ctrlState.prevForce - max_dF, min(ctrlState.prevForce + max_dF, Fx_raw));
    ctrlState.prevForce = Fx_req;
    
    forceCmd.Fx_total = Fx_req;
    
    % (2) ABS modulation
    % Runner가 매 스텝 ctrlState.wheelSlip 에 값을 넣어준다고 가정할 때의 Bang-bang 로직
    forceCmd.brakeRatio = ones(4, 1);
    kappa_target = 0.12;
    
    if ax < 0 && isfield(ctrlState, 'wheelSlip') % 감속 중일 때만 작동
        for i = 1:4
            if abs(ctrlState.wheelSlip(i)) > kappa_target
                forceCmd.brakeRatio(i) = 0.5; % 슬립 초과 시 제동력 50%로 감소
            end
        end
    end
end
