%% [Ch10] 비선형 계산 토크 제어 (Computed Torque Control / Inverse Dynamics)
% =========================================================================
% 설명:
%   본 스크립트는 다자유도 로봇 매니퓰레이터의 비선형 결합 및 원심력/코리올리력을
%   완벽하게 상쇄하여 선형 분리 시스템으로 변환하는 계산 토크 제어(Computed
%   Torque Control / Feedback Linearization)를 2-Link 로봇에 구현하고,
%   고속 궤적(원형/조화 궤적) 추종 성능과 모델 파라미터 불확실성에 대한 강인성을 검증합니다.
%
% 주요 학습 내용:
%   1. 역동역학(Inverse Dynamics) 제어 법칙:
%      tau = M_hat(q) * (q_ddot_des + Kv * e_dot + Kp * e) + C_hat(q, q_dot) * q_dot + G_hat(q)
%   2. 비선형 시스템의 오차 동역학: e_ddot + Kv * e_dot + Kp * e = 0 (이상적인 경우)
%   3. 단순 독립 관절 PD 제어 vs Computed Torque 제어의 궤적 추종 오차 비교
%   4. 모델 파라미터 불확실성(예: 질량 20% 오차) 존재 시의 강인성 평가
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch10] 2자유도 로봇 계산 토크(Computed Torque) 제어 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 로봇 실제 물리 파라미터 (True Model)
real_r.m1 = 2.0; real_r.m2 = 1.5;
real_r.l1 = 1.0; real_r.l2 = 0.8;
real_r.lc1 = 0.5; real_r.lc2 = 0.4;
real_r.I1 = 0.15; real_r.I2 = 0.10;
real_r.g = 9.81;

%% 2. 제어기 내부 추정 모델 (Estimated Model - 15% 불확실성 포함)
model_r = real_r;
model_r.m1 = real_r.m1 * 1.15; % 질량 15% 과대 추정
model_r.m2 = real_r.m2 * 1.20; % 질량 20% 과대 추정

%% 3. 제어기 게인 설정 (임계 감쇠: omega_n = 15 rad/s, zeta = 1.0)
wn = 15;
Kp = diag([wn^2, wn^2]);         % 225
Kv = diag([2*1.0*wn, 2*1.0*wn]); % 30

%% 4. 목표 추종 궤적 정의 (조화 진동 궤적)
t_end = 4.0;
dt = 0.002;
t = 0:dt:t_end;
N = length(t);

% 관절 1, 2의 목표 궤적 [q_d, dq_d, ddq_d]
freq = 1.0; % [Hz]
qd1  =  0.5 * sin(2*pi*freq*t);
dqd1 =  0.5 * (2*pi*freq) * cos(2*pi*freq*t);
ddqd1 = -0.5 * (2*pi*freq)^2 * sin(2*pi*freq*t);

qd2  =  0.4 * cos(2*pi*freq*t);
dqd2 = -0.4 * (2*pi*freq) * sin(2*pi*freq*t);
ddqd2 = -0.4 * (2*pi*freq)^2 * cos(2*pi*freq*t);

%% 5. 시뮬레이션 루프 실행: (A) 단순 독립 PD vs (B) Computed Torque Control
% (A) 단순 PD 제어
res_pd = run_sim('PD', real_r, model_r, Kp, Kv, t, qd1, qd2, dqd1, dqd2, ddqd1, ddqd2);

% (B) Computed Torque 제어
res_ctc = run_sim('CTC', real_r, model_r, Kp, Kv, t, qd1, qd2, dqd1, dqd2, ddqd1, ddqd2);

%% 6. 결과 정량 분석
err_pd_max  = max(sqrt(res_pd.e(1,:).^2 + res_pd.e(2,:).^2));
err_ctc_max = max(sqrt(res_ctc.e(1,:).^2 + res_ctc.e(2,:).^2));

fprintf('[1] 고속 궤적 추종 최대 오차 비교:\n');
fprintf(' - 단순 PD 제어 최대 오차:         %.4f rad (%.2f deg)\n', err_pd_max, rad2deg(err_pd_max));
fprintf(' - Computed Torque 제어 최대 오차: %.4f rad (%.2f deg) -> 오차 약 %.1f%% 감소\n\n', ...
        err_ctc_max, rad2deg(err_ctc_max), (1 - err_ctc_max/err_pd_max)*100);

%% 7. 시각화 (Figure 1: 궤적 추종 및 오차 비교)
figure('Name', 'Ch10: Computed Torque Control vs PD Control', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 80, 1050, 750]);

% (1) Joint 1 추종 궤적
subplot(2, 2, 1);
plot(t, rad2deg(qd1), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired (q_{d1})');
hold on; grid on;
plot(t, rad2deg(res_pd.q(1,:)), 'r:', 'LineWidth', 1.6, 'DisplayName', 'PD Control');
plot(t, rad2deg(res_ctc.q(1,:)), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Computed Torque');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Joint 1 Angle [deg]', 'FontWeight', 'bold');
title('Joint 1 Trajectory Tracking', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

% (2) Joint 2 추종 궤적
subplot(2, 2, 2);
plot(t, rad2deg(qd2), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired (q_{d2})');
hold on; grid on;
plot(t, rad2deg(res_pd.q(2,:)), 'r:', 'LineWidth', 1.6, 'DisplayName', 'PD Control');
plot(t, rad2deg(res_ctc.q(2,:)), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Computed Torque');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Joint 2 Angle [deg]', 'FontWeight', 'bold');
title('Joint 2 Trajectory Tracking', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

% (3) 추종 오차 비교 (Joint 1 & 2)
subplot(2, 2, 3);
plot(t, rad2deg(res_pd.e(1,:)), 'r:', 'LineWidth', 1.6, 'DisplayName', 'PD e_1');
hold on; grid on;
plot(t, rad2deg(res_ctc.e(1,:)), 'b-', 'LineWidth', 1.8, 'DisplayName', 'CTC e_1');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Tracking Error [deg]', 'FontWeight', 'bold');
title('Joint 1 Error (deg)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

% (4) 제어 입력 토크 프로파일
subplot(2, 2, 4);
plot(t, res_ctc.tau(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '\tau_1 (CTC)');
hold on; grid on;
plot(t, res_ctc.tau(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '\tau_2 (CTC)');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Torque [Nm]', 'FontWeight', 'bold');
title('Computed Torque Control Input Profile', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

fprintf(' 계산 토크 제어 시뮬레이션 완료.\n');

%% 시뮬레이션 실행기 함수
function res = run_sim(mode, real_r, model_r, Kp, Kv, t, qd1, qd2, dqd1, dqd2, ddqd1, ddqd2)
    N = length(t);
    dt = t(2) - t(1);
    
    q   = zeros(2, N);
    dq  = zeros(2, N);
    tau = zeros(2, N);
    e   = zeros(2, N);
    
    % 초기 상태: 목표 초기 위치와 일치
    q(:, 1)  = [qd1(1); qd2(1)];
    dq(:, 1) = [dqd1(1); dqd2(1)];
    
    for k = 1:(N - 1)
        qk  = q(:, k);
        dqk = dq(:, k);
        
        q_des   = [qd1(k); qd2(k)];
        dq_des  = [dqd1(k); dqd2(k)];
        ddq_des = [ddqd1(k); ddqd2(k)];
        
        err  = q_des - qk;
        derr = dq_des - dqk;
        e(:, k) = err;
        
        if strcmp(mode, 'PD')
            % 단순 독립 관절 PD 제어
            u = Kp * err + Kv * derr;
            tau_k = u;
        else
            % Computed Torque Control: tau = M_hat * (ddq_d + Kv*derr + Kp*err) + C_hat*dq + G_hat
            [M_hat, C_hat, G_hat] = get_matrices(qk, dqk, model_r);
            v_cmd = ddq_des + Kv * derr + Kp * err;
            tau_k = M_hat * v_cmd + C_hat * dqk + G_hat;
        end
        
        tau(:, k) = tau_k;
        
        % 실제 로봇 동역학 적용: ddq = M_real \ (tau - C_real*dq - G_real)
        [M_real, C_real, G_real] = get_matrices(qk, dqk, real_r);
        ddq_real = M_real \ (tau_k - C_real * dqk - G_real);
        
        % 오일러 적분
        dq(:, k+1) = dqk + ddq_real * dt;
        q(:, k+1)  = qk  + dq(:, k) * dt;
    end
    e(:, N) = [qd1(N); qd2(N)] - q(:, N);
    tau(:, N) = tau(:, N-1);
    
    res.q = q;
    res.dq = dq;
    res.e = e;
    res.tau = tau;
end

%% 동역학 행렬 계산 함수 (M, C, G)
function [M, C, G] = get_matrices(q, dq, r)
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);
    
    m11 = r.I1 + r.I2 + r.m1*r.lc1^2 + r.m2*(r.l1^2 + r.lc2^2 + 2*r.l1*r.lc2*cos(q2));
    m12 = r.I2 + r.m2*(r.lc2^2 + r.l1*r.lc2*cos(q2));
    m21 = m12;
    m22 = r.I2 + r.m2*r.lc2^2;
    M = [m11, m12; m21, m22];
    
    h = -r.m2 * r.l1 * r.lc2 * sin(q2);
    C = [h * dq2,        h * dq1 + h * dq2;
        -h * dq1,        0];
    
    g1 = (r.m1*r.lc1 + r.m2*r.l1)*r.g*cos(q1) + r.m2*r.lc2*r.g*cos(q1 + q2);
    g2 = r.m2*r.lc2*r.g*cos(q1 + q2);
    G = [g1; g2];
end
