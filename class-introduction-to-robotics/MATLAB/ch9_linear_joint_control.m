%% [Ch9] 선형 관절 제어 (Linear Joint Control & Gravity Compensation)
% =========================================================================
% 설명:
%   본 스크립트는 로봇 관절의 독립 제어(Independent Joint Control) 구조를
%   바탕으로, P, PD, PID 제어기의 과도 응답 및 중력 외란에 의한 정상상태 오차를
%   비교 분석하고, 중력 피드포워드(Gravity Feedforward) 보상기를 결합하여
%   고속 정밀 추종 성능을 달성하는 기법을 시뮬레이션합니다.
%
% 주요 학습 내용:
%   1. 단일 관절 동역학 모델: J * q_ddot + b * q_dot + g_torque(q) = tau
%   2. PD 제어기의 특성 및 중력에 의한 드룹(Droop / Steady-State Error)
%   3. PID 제어기의 적분기를 통한 정상상태 오차 제거 (단, 위상 지연/오버슈트 발생)
%   4. PD + Gravity Feedforward 제어기를 통한 신속하고 오차 없는 추종 성능 달성
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch9] 로봇 선형 관절 제어 및 중력 피드포워드 보상 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 관절 물리 파라미터 정의
J = 0.05;      % 등가 관성모멘트 [kg*m^2]
b = 0.1;       % 점성 마찰 계수 [N*m*s/rad]
m_arm = 1.2;   % 링크 질량 [kg]
l_com = 0.4;   % 질량중심 거리 [m]
g_acc = 9.81;  % 중력가속도 [m/s^2]
g_max = m_arm * g_acc * l_com; % 최대 중력 토크 (~4.71 Nm)

% 목표 각도 (계단 입력: 0도 -> 60도)
q_des_deg = 60;
q_des = deg2rad(q_des_deg);

%% 2. 제어기 게인 설정
Kp = 50.0;     % 비례 게인
Kd = 5.0;      % 미분 게인
Ki = 40.0;     % 적분 게인

%% 3. 네 가지 제어 구조 시뮬레이션
% (1) P 제어
% (2) PD 제어
% (3) PID 제어
% (4) PD + Gravity Feedforward 제어
tspan = [0 3.0];
dt = 0.002;
t_vec = tspan(1):dt:tspan(2);
N = length(t_vec);

% 초기 상태 [q; dq; int_e]
res_p     = sim_joint_control('P',  q_des, J, b, g_max, Kp, 0,  0,  t_vec);
res_pd    = sim_joint_control('PD', q_des, J, b, g_max, Kp, Kd, 0,  t_vec);
res_pid   = sim_joint_control('PID',q_des, J, b, g_max, Kp, Kd, Ki, t_vec);
res_pdf_g = sim_joint_control('PD+G', q_des, J, b, g_max, Kp, Kd, 0,  t_vec);

%% 4. 결과 요약 출력
fprintf('[1] 3초 시점 정상상태 오차 비교:\n');
fprintf(' - P 제어기 오차:                 %.2f deg\n', rad2deg(q_des - res_p.q(end)));
fprintf(' - PD 제어기 오차:                %.2f deg\n', rad2deg(q_des - res_pd.q(end)));
fprintf(' - PID 제어기 오차:               %.4f deg (적분기로 오차 제거)\n', rad2deg(q_des - res_pid.q(end)));
fprintf(' - PD + Gravity Feedforward 오차: %.4f deg (완벽 추종)\n\n', rad2deg(q_des - res_pdf_g.q(end)));

%% 5. 시각화 (Figure 1: 각도 응답 및 제어 토크)
figure('Name', 'Ch9: Linear Joint Control & Gravity Compensation', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 100, 1000, 700]);

% (1) 관절 각도 응답
subplot(2, 1, 1);
plot(t_vec, rad2deg(res_p.q), 'r:', 'LineWidth', 1.8, 'DisplayName', 'P Control');
hold on; grid on;
plot(t_vec, rad2deg(res_pd.q), 'm--', 'LineWidth', 1.8, 'DisplayName', 'PD Control');
plot(t_vec, rad2deg(res_pid.q), 'b-', 'LineWidth', 2.0, 'DisplayName', 'PID Control');
plot(t_vec, rad2deg(res_pdf_g.q), 'k-', 'LineWidth', 2.2, 'DisplayName', 'PD + Gravity FF');
yline(q_des_deg, 'k--', 'Target (60^\circ)', 'LineWidth', 1.2);
ylabel('Joint Angle [deg]', 'FontWeight', 'bold');
title('Joint Position Step Response under Gravity Disturbance', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast');

% (2) 제어 입력 토크
subplot(2, 1, 2);
plot(t_vec, res_p.tau, 'r:', 'LineWidth', 1.8, 'DisplayName', 'P');
hold on; grid on;
plot(t_vec, res_pd.tau, 'm--', 'LineWidth', 1.8, 'DisplayName', 'PD');
plot(t_vec, res_pid.tau, 'b-', 'LineWidth', 2.0, 'DisplayName', 'PID');
plot(t_vec, res_pdf_g.tau, 'k-', 'LineWidth', 2.2, 'DisplayName', 'PD + Gravity FF');
ylabel('Control Torque [Nm]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('Control Input Torque Profile', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast');

fprintf(' 선형 관절 제어 시뮬레이션 완료.\n');

%% 시뮬레이션 보조 루프 함수
function res = sim_joint_control(ctrl_type, q_des, J, b, g_max, Kp, Kd, Ki, t_vec)
    N = length(t_vec);
    dt = t_vec(2) - t_vec(1);
    
    q = zeros(N, 1);
    dq = zeros(N, 1);
    tau = zeros(N, 1);
    int_e = 0;
    
    for k = 1:(N - 1)
        err = q_des - q(k);
        derr = 0 - dq(k);
        int_e = int_e + err * dt;
        
        % 중력 토크: tau_g = g_max * cos(q) (수평 기준)
        tau_gravity = g_max * cos(q(k));
        
        % 제어 입력 계산
        switch ctrl_type
            case 'P'
                u = Kp * err;
            case 'PD'
                u = Kp * err + Kd * derr;
            case 'PID'
                u = Kp * err + Kd * derr + Ki * int_e;
            case 'PD+G'
                u = Kp * err + Kd * derr + tau_gravity; % 중력 피드포워드 결합
        end
        
        tau(k) = u;
        
        % 관절 동역학: J * ddq + b * dq + tau_gravity = u
        ddq = (u - b * dq(k) - tau_gravity) / J;
        
        % 오일러 수치 적분
        dq(k + 1) = dq(k) + ddq * dt;
        q(k + 1)  = q(k) + dq(k) * dt;
    end
    tau(N) = tau(N - 1);
    
    res.q = q;
    res.dq = dq;
    res.tau = tau;
end
