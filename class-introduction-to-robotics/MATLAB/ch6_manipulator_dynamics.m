%% [Ch6] 매니퓰레이터 동역학 (Manipulator Dynamics & Euler-Lagrange)
% =========================================================================
% 설명:
%   본 스크립트는 2자유도 평면 로봇 팔의 Euler-Lagrange 비선형 동역학
%   방정식 M(q)*q_ddot + C(q, q_dot)*q_dot + G(q) = tau 를 구성하고,
%   외부 토크가 인가되지 않은 상태(자유 낙하/진자 운동)에서의 비선형 미분방정식을
%   ode45를 통해 수치 적분하여 시뮬레이션하고 에너지 보존 법칙을 검증합니다.
%
% 주요 학습 내용:
%   1. 관성 행렬 M(q) (Symmetric, Positive-Definite) 구성
%   2. 코리올리 및 원심력 행렬 C(q, q_dot) (M_dot - 2C 는 Skew-Symmetric 특성)
%   3. 중력 토크 벡터 G(q) 유도
%   4. ode45를 이용한 비선형 상태 공간 시뮬레이션: x_dot = [q_dot; M^-1*(tau - C*q_dot - G)]
%   5. 시스템 총 에너지(운동 에너지 T + 위치 에너지 V) 보존성 검증
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch6] 2자유도 로봇 매니퓰레이터 비선형 동역학 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 로봇 물리 파라미터 정의
robot.m1 = 2.0;    % 링크 1 질량 [kg]
robot.m2 = 1.5;    % 링크 2 질량 [kg]
robot.l1 = 1.0;    % 링크 1 전체 길이 [m]
robot.l2 = 0.8;    % 링크 2 전체 길이 [m]
robot.lc1 = 0.5;   % 링크 1 질량중심 위치 [m]
robot.lc2 = 0.4;   % 링크 2 질량중심 위치 [m]
robot.I1 = 0.15;   % 링크 1 관성모멘트 [kg*m^2]
robot.I2 = 0.10;   % 링크 2 관성모멘트 [kg*m^2]
robot.g = 9.81;    % 중력가속도 [m/s^2]

fprintf('[1] 물리 파라미터:\n');
fprintf(' - m1 = %.1f kg, l1 = %.1f m, I1 = %.2f kg*m^2\n', robot.m1, robot.l1, robot.I1);
fprintf(' - m2 = %.1f kg, l2 = %.1f m, I2 = %.2f kg*m^2\n\n', robot.m2, robot.l2, robot.I2);

%% 2. 시뮬레이션 조건 설정
tspan = [0 5];     % 시뮬레이션 시간: 0 ~ 5초

% 초기 상태: [theta1, theta2, theta1_dot, theta2_dot]
% 수평 상태(90도, 0도)에서 정지 상태로 놓았을 때의 자유 진자 운동
x0 = [deg2rad(90); deg2rad(0); 0; 0];

%% 3. 비선형 미분방정식 수치 적분 (ode45)
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, x] = ode45(@(t, x) robot_ode(t, x, robot), tspan, x0, options);

q1 = x(:, 1);
q2 = x(:, 2);
dq1 = x(:, 3);
dq2 = x(:, 4);

%% 4. 시스템 에너지 계산 및 보존성 검증
N = length(t);
E_kin = zeros(N, 1);
E_pot = zeros(N, 1);

for k = 1:N
    qk = [q1(k); q2(k)];
    dqk = [dq1(k); dq2(k)];
    
    [M, ~, G, V] = get_dynamics_matrices(qk, dqk, robot);
    
    % 운동에너지: T = 1/2 * dq^T * M(q) * dq
    E_kin(k) = 0.5 * dqk' * M * dqk;
    % 위치에너지: V(q)
    E_pot(k) = V;
end

E_total = E_kin + E_pot;
energy_drift = max(abs(E_total - E_total(1)));

fprintf('[2] 에너지 보존성 검증:\n');
fprintf(' - 초기 총 에너지 E_0 = %.4f J\n', E_total(1));
fprintf(' - 최대 에너지 변동 편차: %.2e J (보존 법칙 성립)\n\n', energy_drift);

%% 5. 시뮬레이션 결과 시각화
figure('Name', 'Ch6: Manipulator Dynamics & Energy Conservation', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [150, 100, 1000, 700]);

% (1) 관절 각도 응답
subplot(2, 2, 1);
plot(t, rad2deg(q1), 'b-', 'LineWidth', 2, 'DisplayName', '\theta_1 (Link 1)');
hold on; grid on;
plot(t, rad2deg(q2), 'r--', 'LineWidth', 2, 'DisplayName', '\theta_2 (Link 2)');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Joint Angles [deg]', 'FontWeight', 'bold');
title('Joint Angles (Free Fall Oscillation)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

% (2) 관절 각속도 응답
subplot(2, 2, 2);
plot(t, dq1, 'b-', 'LineWidth', 2, 'DisplayName', 'd\theta_1/dt');
hold on; grid on;
plot(t, dq2, 'r--', 'LineWidth', 2, 'DisplayName', 'd\theta_2/dt');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Joint Velocities [rad/s]', 'FontWeight', 'bold');
title('Joint Velocities', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');

% (3) 에너지 보존 그래프
subplot(2, 2, [3, 4]);
plot(t, E_kin, 'g-', 'LineWidth', 1.8, 'DisplayName', 'Kinetic Energy (T)');
hold on; grid on;
plot(t, E_pot, 'm-', 'LineWidth', 1.8, 'DisplayName', 'Potential Energy (V)');
plot(t, E_total, 'k--', 'LineWidth', 2.5, 'DisplayName', 'Total Mechanical Energy (E_{tot})');
xlabel('Time [s]', 'FontWeight', 'bold'); ylabel('Energy [Joules]', 'FontWeight', 'bold');
title(sprintf('Mechanical Energy Conservation (Drift: %.2e J)', energy_drift), ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'east');

fprintf(' 동역학 시뮬레이션 및 에너지 보존성 분석 완료.\n');

%% 로봇 상태 방정식 함수 (ODE)
function dxdt = robot_ode(~, x, robot)
    q = x(1:2);
    dq = x(3:4);
    
    [M, C, G, ~] = get_dynamics_matrices(q, dq, robot);
    
    % 토크 입력: 자유 진자이므로 tau = [0; 0]
    tau = [0; 0];
    
    % 가속도 계산: q_ddot = M \ (tau - C*dq - G)
    ddq = M \ (tau - C*dq - G);
    
    dxdt = [dq; ddq];
end

%% 동역학 행렬 계산 함수 (M, C, G, V)
function [M, C, G, V] = get_dynamics_matrices(q, dq, r)
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);
    
    % 1. 질량/관성 행렬 M(q)
    m11 = r.I1 + r.I2 + r.m1*r.lc1^2 + r.m2*(r.l1^2 + r.lc2^2 + 2*r.l1*r.lc2*cos(q2));
    m12 = r.I2 + r.m2*(r.lc2^2 + r.l1*r.lc2*cos(q2));
    m21 = m12;
    m22 = r.I2 + r.m2*r.lc2^2;
    M = [m11, m12; m21, m22];
    
    % 2. 코리올리 및 원심력 행렬 C(q, dq) (Christoffel Symbols)
    h = -r.m2 * r.l1 * r.lc2 * sin(q2);
    C = [h * dq2,        h * dq1 + h * dq2;
        -h * dq1,        0];
    
    % 3. 중력 토크 벡터 G(q)
    g1 = (r.m1*r.lc1 + r.m2*r.l1)*r.g*cos(q1) + r.m2*r.lc2*r.g*cos(q1 + q2);
    g2 = r.m2*r.lc2*r.g*cos(q1 + q2);
    G = [g1; g2];
    
    % 4. 위치에너지 V(q)
    V = (r.m1*r.lc1 + r.m2*r.l1)*r.g*sin(q1) + r.m2*r.lc2*r.g*sin(q1 + q2);
end
