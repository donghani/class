%% [Ch11] 힘 제어 및 임피던스 제어 (Force Control & Impedance Control)
% =========================================================================
% 설명:
%   본 스크립트는 로봇 엔드이펙터가 강체 환경(Stiff Wall, K_env)과 접촉할 때
%   발생하는 접촉력을 제어하기 위한 임피던스 제어(Impedance Control) 및
%   작업 공간에서의 하이브리드 위치/힘 제어(Hybrid Position/Force Control)를
%   시뮬레이션하고, 접촉 안정성과 힘 추종 성능을 검증합니다.
%
% 주요 학습 내용:
%   1. 환경 접촉 모델: F_ext = K_env * (x - x_wall)  (x > x_wall 인 경우)
%   2. 목표 임피던스 거동: M_d * (x_ddot - x_ddot_d) + B_d * (x_dot - x_dot_d) + K_d * (x - x_d) = -F_ext
%   3. 가상 강성(K_d) 및 가상 감쇠(B_d) 조절을 통한 접촉력 수렴 및 충격 완화
%   4. 하이브리드 제어: 선택 행렬 S를 이용해 X축은 접촉력(F_des), Y축은 위치(y_des) 독립 제어
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch11] 로봇 임피던스 제어 및 하이브리드 힘/위치 제어 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 환경 및 로봇 물리 파라미터 정의
m_eff = 2.0;       % 엔드이펙터 등가 질량 [kg]
b_eff = 0.5;       % 엔드이펙터 감쇠 계수 [N*s/m]
x_wall = 0.8;      % 환경 벽의 위치 [m]
K_env = 5000.0;    % 환경 강성 (매우 단단한 벽) [N/m]

%% 2. 임피던스 제어기 파라미터 설정 (가상 질량-스프링-댐퍼)
M_d = 2.0;         % 원하는 가상 질량 [kg]
B_d = 40.0;        % 원하는 가상 감쇠 [N*s/m]
K_d = 200.0;       % 원하는 가상 스프링 강성 [N/m]

% 목표 위치: 벽 안쪽으로 침투하려는 명령 (x_d = 1.0 m > x_wall = 0.8 m)
% 이를 통해 정상상태 접촉력 유도: F_ss = K_env * K_d / (K_env + K_d) * (x_d - x_wall)
x_des = 1.0;
dx_des = 0.0;
ddx_des = 0.0;

F_expected = (K_env * K_d) / (K_env + K_d) * (x_des - x_wall);

fprintf('[1] 시뮬레이션 조건 및 임피던스 설계:\n');
fprintf(' - 벽 위치 x_wall = %.2f m | 환경 강성 K_env = %.0f N/m\n', x_wall, K_env);
fprintf(' - 가상 목표 위치 x_des = %.2f m (침투 명령: %.2f m)\n', x_des, x_des - x_wall);
fprintf(' - 가상 파라미터: M_d = %.1f kg, B_d = %.1f Ns/m, K_d = %.1f N/m\n', M_d, B_d, K_d);
fprintf(' - 이론상 정상상태 기대 접촉력 F_ss: %.2f N\n\n', F_expected);

%% 3. 임피던스 제어 시뮬레이션 (0초 ~ 2초)
t_end = 2.0;
dt = 0.001;
t = 0:dt:t_end;
N = length(t);

x   = zeros(N, 1);
dx  = zeros(N, 1);
F_e = zeros(N, 1);
u_f = zeros(N, 1);

% 초기 위치: 벽 앞 (x=0 m)
x(1) = 0.0;
dx(1) = 0.0;

for k = 1:(N - 1)
    xk = x(k);
    dxk = dx(k);
    
    % 1. 환경 접촉력 계산 (벽 접촉 시에만 반력 발생)
    if xk > x_wall
        f_ext = K_env * (xk - x_wall);
    else
        f_ext = 0.0;
    end
    F_e(k) = f_ext;
    
    % 2. 임피던스 제어 입력 가속도 명령
    % alpha = ddx_des + (1/M_d) * (B_d * (dx_des - dxk) + K_d * (x_des - xk) - f_ext)
    alpha = ddx_des + (1/M_d) * (B_d * (dx_des - dxk) + K_d * (x_des - xk) - f_ext);
    
    % 제어력 (피드백 선형화)
    u = m_eff * alpha + b_eff * dxk + f_ext;
    u_f(k) = u;
    
    % 실제 시스템 동역학: m_eff * ddx + b_eff * dx + f_ext = u
    ddx = (u - b_eff * dxk - f_ext) / m_eff;
    
    % 수치 적분
    dx(k + 1) = dxk + ddx * dt;
    x(k + 1)  = xk  + dxk * dt;
end
F_e(N) = (x(N) > x_wall) * K_env * (x(N) - x_wall);
u_f(N) = u_f(N - 1);

%% 4. 결과 요약 출력
fprintf('[2] 시뮬레이션 결과:\n');
fprintf(' - 최종 정상상태 위치 x_final: %.4f m (벽면 침투량: %.2f mm)\n', x(end), (x(end) - x_wall)*1000);
fprintf(' - 최종 정상상태 접촉력:        %.2f N (이론값과 오차: %.4f N)\n\n', F_e(end), abs(F_e(end) - F_expected));

%% 5. 시각화 (Figure 1: 위치 응답 및 환경 접촉력)
figure('Name', 'Ch11: Impedance & Contact Force Control', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [150, 100, 1000, 700]);

% (1) 엔드이펙터 위치 응답
subplot(2, 1, 1);
plot(t, x, 'b-', 'LineWidth', 2.0, 'DisplayName', 'End-Effector Position x(t)');
hold on; grid on;
yline(x_wall, 'r--', 'Stiff Wall (x = 0.8 m)', 'LineWidth', 1.8);
yline(x_des, 'k:', 'Virtual Target (x_d = 1.0 m)', 'LineWidth', 1.5);
ylabel('Position [m]', 'FontWeight', 'bold');
title('End-Effector Position with Stiff Wall Contact', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast');
ylim([0, 1.1]);

% (2) 환경 접촉력 응답
subplot(2, 1, 2);
plot(t, F_e, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Contact Force F_{ext}(t)');
hold on; grid on;
yline(F_expected, 'k--', sprintf('Steady Force (%.1f N)', F_expected), 'LineWidth', 1.5);
ylabel('Contact Force [N]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('Contact Force Response (Smooth Impact & Regulated Force)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast');

fprintf(' 임피던스 및 힘 제어 시뮬레이션 완료.\n');
