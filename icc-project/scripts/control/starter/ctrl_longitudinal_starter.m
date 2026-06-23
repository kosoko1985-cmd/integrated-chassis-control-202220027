function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
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
