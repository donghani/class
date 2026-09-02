%% [Ch8] 기구 설계 및 액추에이터 사이징 (Mechanism Design & Sizing)
% =========================================================================
% 설명:
%   본 스크립트는 로봇 매니퓰레이터 설계 시 고려해야 할 핵심 요소인
%   모터 및 감속기(Gear Ratio N) 관성 매칭(Inertia Matching),
%   부하에 따른 모터 요구 토크 사이징 곡선, 링크 길이 비율에 따른
%   작업 공간(Workspace) 면적 및 가조작성 최적화 설계를 다룹니다.
%
% 주요 학습 내용:
%   1. 감속비 N에 따른 모터 축 등가 관성: J_eq = J_m + J_L / N^2
%   2. 관성 매칭 조건: 부하 가속도를 최대화하는 최적 감속비 N_opt = sqrt(J_L / J_m)
%   3. 모터 요구 토크 tau_m = tau_L / N + (J_m + J_L / N^2) * N * theta_ddot
%   4. 링크 길이 비율 (l2 / l1)에 따른 2링크 로봇 작업 영역 및 Dexterous Area 비교
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch8] 로봇 기구 설계 및 모터/감속기 사이징 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 모터 및 부하 파라미터 정의
J_m = 0.0005;    % 모터 로터 자체 관성모멘트 [kg*m^2]
J_L = 0.8;       % 로봇 링크 및 페이로드 부하 관성모멘트 [kg*m^2]
tau_L = 15.0;    % 정적 부하 토크 (중력, 마찰) [Nm]
theta_L_dd = 25; % 요구 부하 각가속도 [rad/s^2]

%% 2. 관성 매칭(Inertia Matching) 최적 감속비 계산
% N_opt = sqrt(J_L / J_m)
N_opt = sqrt(J_L / J_m);

fprintf('[1] 관성 매칭 분석:\n');
fprintf(' - 모터 로터 관성 J_m: %.6f kg*m^2\n', J_m);
fprintf(' - 부하 관성 J_L:       %.4f kg*m^2\n', J_L);
fprintf(' - 최적 관성 매칭 감속비 N_opt: %.2f : 1\n\n', N_opt);

%% 3. 감속비 N에 따른 모터 요구 토크 및 부하 가속 성능 분석
N_range = linspace(5, 100, 300);

% 모터 축에서 바라본 요구 토크
% tau_m = tau_L / N + (J_m * N + J_L / N) * theta_L_dd
tau_motor = (tau_L ./ N_range) + (J_m .* N_range + J_L ./ N_range) * theta_L_dd;

% 단위 모터 토크 당 부하 가속도 (가속 성능 효율)
acc_eff = N_range ./ (J_m .* N_range.^2 + J_L);

%% 4. 링크 길이 비율(l2 / l1)에 따른 작업 공간 최적화 분석
L_total = 1.0; % 총 링크 길이 고정 (l1 + l2 = 1.0 m)
r_ratio = linspace(0.1, 0.9, 100);
ws_area = zeros(size(r_ratio));

for k = 1:length(r_ratio)
    l1_k = L_total * (1 - r_ratio(k));
    l2_k = L_total * r_ratio(k);
    
    r_outer = l1_k + l2_k;
    r_inner = abs(l1_k - l2_k);
    
    % 평면 도넛형 작업 공간 면적: Area = pi * (r_outer^2 - r_inner^2)
    ws_area(k) = pi * (r_outer^2 - r_inner^2);
end

%% 5. 시각화 (Figure 1 & 2)
figure('Name', 'Ch8: Mechanism Sizing & Inertia Matching', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [120, 80, 1000, 650]);

% (1) 감속비에 따른 모터 요구 토크 곡선
subplot(2, 2, 1);
plot(N_range, tau_motor, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(N_opt, (tau_L / N_opt) + (J_m * N_opt + J_L / N_opt) * theta_L_dd, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Gear Ratio (N)', 'FontWeight', 'bold');
ylabel('Required Motor Torque [Nm]', 'FontWeight', 'bold');
title('Required Motor Torque vs. Gear Ratio', 'FontSize', 11, 'FontWeight', 'bold');
legend('Required Torque', sprintf('N_{opt} = %.1f', N_opt), 'Location', 'best');

% (2) 단위 토크당 가속 효율 곡선 (관성 매칭 최적점)
subplot(2, 2, 2);
plot(N_range, acc_eff, 'g-', 'LineWidth', 2);
hold on; grid on;
plot(N_opt, N_opt / (J_m * N_opt^2 + J_L), 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Gear Ratio (N)', 'FontWeight', 'bold');
ylabel('Acceleration Efficiency [rad/s^2 / Nm]', 'FontWeight', 'bold');
title('Acceleration Efficiency vs. Gear Ratio', 'FontSize', 11, 'FontWeight', 'bold');
legend('Acceleration Efficiency', 'Peak (Inertia Match)', 'Location', 'best');

% (3) 링크 길이 비율에 따른 작업 공간 면적
subplot(2, 2, [3, 4]);
plot(r_ratio, ws_area, 'm-', 'LineWidth', 2.2);
hold on; grid on;
[max_area, max_idx] = max(ws_area);
plot(r_ratio(max_idx), max_area, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
xlabel('Link Ratio (l_2 / (l_1 + l_2))', 'FontWeight', 'bold');
ylabel('Workspace Area [m^2]', 'FontWeight', 'bold');
title(sprintf('Workspace Reach Area vs. Link Length Ratio (Optimal l_1 = l_2 at Ratio = 0.5)'), ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('Workspace Area', 'Maximum Reachable Area (l_1 = l_2)', 'Location', 'south');

fprintf(' 기구 설계 및 사이징 해석 완료.\n');
