%% [Ch7] 궤적 생성 및 경로 계획 (Trajectory Generation)
% =========================================================================
% 설명:
%   본 스크립트는 로봇의 관절 공간 및 작업 공간에서의 세 가지 핵심 궤적 생성
%   알고리즘(3차 다항식 Cubic, 5차 다항식 Quintic, 사다리꼴 속도 프로파일 LSPB)을
%   구현하고, 위치, 속도, 가속도, 저크(Jerk) 프로파일을 종합적으로 비교 분석합니다.
%
% 주요 학습 내용:
%   1. 3차 다항식(Cubic Polynomial) 궤적 생성 (위치, 속도 경계조건 만족)
%   2. 5차 다항식(Quintic Polynomial) 궤적 생성 (가속도 경계조건 포함, 저크 완화)
%   3. 사다리꼴 속도 궤적 (LSPB - Linear Segment with Parabolic Blend)
%   4. 가속도 불연속에 따른 저크(Jerk) 비교 및 부드러운 로봇 모션 설계 원리
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch7] 로봇 모션 궤적 생성 (Trajectory Generation) 비교 분석\n');
fprintf('=================================================================\n\n');

%% 1. 궤적 경계 조건 정의
q0 = 10.0;    % 시작 위치 [deg]
qf = 80.0;    % 목표 위치 [deg]
v0 = 0.0;     % 시작 속도 [deg/s]
vf = 0.0;     % 종료 속도 [deg/s]
a0 = 0.0;     % 시작 가속도 [deg/s^2]
af = 0.0;     % 종료 가속도 [deg/s^2]
tf = 2.0;     % 이동 완료 시간 [s]

t = linspace(0, tf, 200);
dt = t(2) - t(1);

%% 2. 3차 다항식 (Cubic Polynomial) 궤적 생성
% q(t) = a0 + a1*t + a2*t^2 + a3*t^3
% 경계 조건: q(0)=q0, q(tf)=qf, dq(0)=v0, dq(tf)=vf
M_cubic = [
    1,  0,    0,      0;
    0,  1,    0,      0;
    1, tf, tf^2,   tf^3;
    0,  1, 2*tf, 3*tf^2
];
b_cubic = [q0; v0; qf; vf];
a_cub = M_cubic \ b_cubic;

q_cub  = a_cub(1) + a_cub(2)*t + a_cub(3)*t.^2 + a_cub(4)*t.^3;
dq_cub = a_cub(2) + 2*a_cub(3)*t + 3*a_cub(4)*t.^2;
ddq_cub = 2*a_cub(3) + 6*a_cub(4)*t;
jerk_cub = 6*a_cub(4) * ones(size(t));

%% 3. 5차 다항식 (Quintic Polynomial) 궤적 생성
% q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
M_quin = [
    1,  0,    0,      0,       0,        0;
    0,  1,    0,      0,       0,        0;
    0,  0,    2,      0,       0,        0;
    1, tf, tf^2,   tf^3,    tf^4,     tf^5;
    0,  1, 2*tf, 3*tf^2,  4*tf^3,   5*tf^4;
    0,  0,    2,   6*tf, 12*tf^2,  20*tf^3
];
b_quin = [q0; v0; a0; qf; vf; af];
a_quin = M_quin \ b_quin;

q_quin  = a_quin(1) + a_quin(2)*t + a_quin(3)*t.^2 + a_quin(4)*t.^3 + a_quin(5)*t.^4 + a_quin(6)*t.^5;
dq_quin = a_quin(2) + 2*a_quin(3)*t + 3*a_quin(4)*t.^2 + 4*a_quin(5)*t.^3 + 5*a_quin(6)*t.^4;
ddq_quin = 2*a_quin(3) + 6*a_quin(4)*t + 12*a_quin(5)*t.^2 + 20*a_quin(6)*t.^3;
jerk_quin = 6*a_quin(4) + 24*a_quin(5)*t + 60*a_quin(6)*t.^2;

%% 4. 사다리꼴 속도 프로파일 (LSPB - Trapezoidal)
% 가속 시간 tb (예: 전체 시간의 25%)
tb = 0.5; % [s]
V_lspb = (qf - q0) / (tf - tb); % 등속 구간 속도
acc_lspb = V_lspb / tb;         % 가속도

q_lspb = zeros(size(t));
dq_lspb = zeros(size(t));
ddq_lspb = zeros(size(t));

for i = 1:length(t)
    ti = t(i);
    if ti <= tb
        % 1단계: 등가속 구간
        q_lspb(i) = q0 + 0.5 * acc_lspb * ti^2;
        dq_lspb(i) = acc_lspb * ti;
        ddq_lspb(i) = acc_lspb;
    elseif ti <= (tf - tb)
        % 2단계: 등속 구간
        q_lspb(i) = q0 + 0.5 * acc_lspb * tb^2 + V_lspb * (ti - tb);
        dq_lspb(i) = V_lspb;
        ddq_lspb(i) = 0;
    else
        % 3단계: 등감속 구간
        t_rem = tf - ti;
        q_lspb(i) = qf - 0.5 * acc_lspb * t_rem^2;
        dq_lspb(i) = acc_lspb * t_rem;
        ddq_lspb(i) = -acc_lspb;
    end
end
jerk_lspb = gradient(ddq_lspb, dt);

fprintf('[1] 궤적 생성 파라미터 요약:\n');
fprintf(' - 이동 변위: %.1f deg -> %.1f deg (Delta = %.1f deg)\n', q0, qf, qf - q0);
fprintf(' - 3차 최대 속도: %.2f deg/s | 최대 가속도: %.2f deg/s^2\n', max(abs(dq_cub)), max(abs(ddq_cub)));
fprintf(' - 5차 최대 속도: %.2f deg/s | 최대 가속도: %.2f deg/s^2\n', max(abs(dq_quin)), max(abs(ddq_quin)));
fprintf(' - 사다리꼴(LSPB) 등속: %.2f deg/s | 가속도: %.2f deg/s^2\n\n', V_lspb, acc_lspb);

%% 5. 시각화 (4x1 서브플롯: Position, Velocity, Acceleration, Jerk)
figure('Name', 'Ch7: Trajectory Comparison (Cubic vs Quintic vs LSPB)', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 50, 950, 850]);

% (1) 위치 (Position)
subplot(4, 1, 1);
plot(t, q_cub, 'b-', 'LineWidth', 2, 'DisplayName', 'Cubic (3rd)');
hold on; grid on;
plot(t, q_quin, 'r--', 'LineWidth', 2, 'DisplayName', 'Quintic (5th)');
plot(t, q_lspb, 'g-.', 'LineWidth', 2, 'DisplayName', 'LSPB (Trapezoidal)');
ylabel('Position [deg]', 'FontWeight', 'bold');
title('Trajectory Profiles Comparison', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest');

% (2) 속도 (Velocity)
subplot(4, 1, 2);
plot(t, dq_cub, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(t, dq_quin, 'r--', 'LineWidth', 2);
plot(t, dq_lspb, 'g-.', 'LineWidth', 2);
ylabel('Velocity [deg/s]', 'FontWeight', 'bold');

% (3) 가속도 (Acceleration)
subplot(4, 1, 3);
plot(t, ddq_cub, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(t, ddq_quin, 'r--', 'LineWidth', 2);
plot(t, ddq_lspb, 'g-.', 'LineWidth', 2);
ylabel('Accel [deg/s^2]', 'FontWeight', 'bold');

% (4) 저크 (Jerk)
subplot(4, 1, 4);
plot(t, jerk_cub, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(t, jerk_quin, 'r--', 'LineWidth', 2);
plot(t, jerk_lspb, 'g-.', 'LineWidth', 1.5);
ylabel('Jerk [deg/s^3]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');

fprintf(' 궤적 생성 및 프로파일 비교 시각화 완료.\n');
