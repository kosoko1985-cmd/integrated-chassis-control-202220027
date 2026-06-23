function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% TODO: 학생 구현
   %% TODO: 학생 구현
    
    dampingCmd = zeros(4, 1);
    
    for i = 1:4
        zs_dot = suspState.zs_dot(i);
        zu_dot = suspState.zu_dot(i);
        v_rel = zs_dot - zu_dot;
        
        % (1) skyhook (On-Off Semi-active 방식 적용)
        % Sprung mass의 절대 속도와 상대 속도의 부호가 같을 때 감쇠력 증가
        if (zs_dot * v_rel) > 0
            % (2) per-wheel 적용: 요구 감쇠력 계산
            % 0 나누기 방지를 위해 분모에 작은 값(eps) 추가
            c_req = CTRL.VER.skyGain * abs(zs_dot / (v_rel + 1e-6));
        else
            % 방향이 반대면 승차감을 위해 최소 감쇠력 적용
            c_req = CTRL.VER.cMin;
        end
        
        % (3) cMin/cMax 제한
        dampingCmd(i) = max(CTRL.VER.cMin, min(CTRL.VER.cMax, c_req));
    end
end

end
