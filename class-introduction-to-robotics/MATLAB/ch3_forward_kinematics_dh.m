%% [Ch3] 순기구학 및 DH 파라미터 (Forward Kinematics & DH Parameters)
% =========================================================================
% 설명:
%   본 스크립트는 표준 Denavit-Hartenberg (DH) 파라미터(Craig 기법)를 활용하여
%   다자유도 로봇 매니퓰레이터의 순기구학(Forward Kinematics)을 계산하고,
%   3차원 공간에서 각 링크와 조인트의 자세 및 엔드이펙터 위치를 렌더링합니다.
%
% 주요 학습 내용:
%   1. DH 파라미터 4요소: link length(a), link twist(alpha), link offset(d), joint angle(theta)
%   2. 인접 좌표계 간 변환 행렬 T_i^{i-1} 계산 함수
%   3. 전체 동차 변환 행렬 T_0^n 계산 및 엔드이펙터 위치/자세 추출
%   4. 3자유도 RRR 로봇 팔 3D 모델링 및 다양한 관절각에 따른 포즈 시각화
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch3] DH 파라미터 기반 순기구학(Forward Kinematics) 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 로봇 링크 파라미터 정의 (3-DOF Spatial Arm)
% 링크 길이 (m)
L1 = 0.5;   % 베이스 높이 (d1)
L2 = 0.8;   % 링크 2 길이 (a2)
L3 = 0.6;   % 링크 3 길이 (a3)

% 목표 관절 각도 (입력값) [deg]
q_deg = [30, 45, -30];
q = deg2rad(q_deg);

%% 2. Craig 표준 DH 테이블 구성
% [a_{i-1}, alpha_{i-1}, d_i, theta_i]
% Craig 컨벤션: T_i^{i-1} = RotX(alpha_{i-1}) * TransX(a_{i-1}) * RotZ(theta_i) * TransZ(d_i)
DH_table = [
    0,   0,       L1, q(1);    % Joint 1 (Z0축 회전)
    0,   pi/2,    0,  q(2);    % Joint 2 (Z1축 회전, X1축 기준 90도 비틀림)
    L2,  0,       0,  q(3);    % Joint 3 (Z2축 회전, 링크2 길이 a2)
    L3,  0,       0,  0        % End-Effector Frame
];

fprintf('[1] DH 파라미터 테이블 (Craig 표기법):\n');
fprintf(' Link | a_{i-1} (m) | alpha_{i-1} (rad) | d_i (m) | theta_i (rad)\n');
fprintf('------------------------------------------------------------\n');
for i = 1:size(DH_table, 1)
    fprintf('  %d   |   %7.3f   |      %7.3f      | %7.3f |   %7.3f\n', ...
        i, DH_table(i,1), DH_table(i,2), DH_table(i,3), DH_table(i,4));
end
fprintf('\n');

%% 3. 각 좌표계 변환 행렬 및 누적 변환 계산
num_frames = size(DH_table, 1);
T_all = zeros(4, 4, num_frames + 1);
T_all(:,:,1) = eye(4); % 베이스 좌표계 {0}

for i = 1:num_frames
    a     = DH_table(i, 1);
    alpha = DH_table(i, 2);
    d     = DH_table(i, 3);
    th    = DH_table(i, 4);
    
    % Craig DH 변환 행렬
    T_rel = [
        cos(th),               -sin(th),               0,              a;
        sin(th)*cos(alpha),   cos(th)*cos(alpha),   -sin(alpha),    -sin(alpha)*d;
        sin(th)*sin(alpha),   cos(th)*sin(alpha),    cos(alpha),     cos(alpha)*d;
        0,                    0,                     0,              1
    ];

    T_all(:,:,i+1) = T_all(:,:,i) * T_rel;
end

% 최종 엔드이펙터 변환 행렬
T_0_EE = T_all(:,:,end);
p_EE = T_0_EE(1:3, 4);
R_EE = T_0_EE(1:3, 1:3);

fprintf('[2] 최종 엔드이펙터 위치 및 자세:\n');
fprintf(' - End-Effector 위치 (X, Y, Z): [%.4f, %.4f, %.4f] m\n', p_EE(1), p_EE(2), p_EE(3));
fprintf(' - End-Effector 변환 행렬 T_0^EE:\n');
disp(T_0_EE);

%% 4. 3D 로봇 매니퓰레이터 시각화
figure('Name', 'Ch3: Forward Kinematics 3D Rendering', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [150, 100, 950, 750]);
hold on; grid on; axis equal;
view(50, 25);
xlabel('X [m]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Z [m]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('3-DOF Spatial Manipulator Forward Kinematics\n[\\theta_1, \\theta_2, \\theta_3] = [%d^\\circ, %d^\\circ, %d^\\circ]', ...
      q_deg(1), q_deg(2), q_deg(3)), 'FontSize', 13, 'FontWeight', 'bold');

% 관절 위치 추출
joint_pos = zeros(3, num_frames + 1);
for i = 1:(num_frames + 1)
    joint_pos(:, i) = T_all(1:3, 4, i);
end

% 로봇 링크 그리기 (두꺼운 선 및 실린더 스타일)
plot3(joint_pos(1, :), joint_pos(2, :), joint_pos(3, :), '-o', ...
      'Color', [0.2 0.4 0.8], 'LineWidth', 5, 'MarkerSize', 8, ...
      'MarkerFaceColor', [1 0.6 0.1], 'MarkerEdgeColor', 'k');

% 베이스 베이스 플랫폼 작도
theta_base = linspace(0, 2*pi, 30);
r_base = 0.25;
patch(r_base*cos(theta_base), r_base*sin(theta_base), zeros(size(theta_base)), ...
      [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'k');

% 각 프레임 좌표계 축 작도 (RGB: X, Y, Z)
axis_len = 0.25;
for i = 1:(num_frames + 1)
    T_curr = T_all(:, :, i);
    origin = T_curr(1:3, 4);
    R_curr = T_curr(1:3, 1:3);
    
    quiver3(origin(1), origin(2), origin(3), R_curr(1,1)*axis_len, R_curr(2,1)*axis_len, R_curr(3,1)*axis_len, 0, 'r', 'LineWidth', 1.8);
    quiver3(origin(1), origin(2), origin(3), R_curr(1,2)*axis_len, R_curr(2,2)*axis_len, R_curr(3,2)*axis_len, 0, 'g', 'LineWidth', 1.8);
    quiver3(origin(1), origin(2), origin(3), R_curr(1,3)*axis_len, R_curr(2,3)*axis_len, R_curr(3,3)*axis_len, 0, 'b', 'LineWidth', 1.8);
    
    if i <= num_frames
        text(origin(1), origin(2), origin(3)+0.06, sprintf('J_%d', i), 'FontSize', 10, 'FontWeight', 'bold');
    else
        text(origin(1), origin(2), origin(3)+0.06, 'EE', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'm');
    end
end

% 엔드이펙터 강조
plot3(p_EE(1), p_EE(2), p_EE(3), 'p', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');

xlim([-1.2, 1.5]); ylim([-1.2, 1.5]); zlim([0, 1.8]);
hold off;

fprintf(' 순기구학 계산 및 3D 로봇 렌더링이 완료되었습니다.\n');
