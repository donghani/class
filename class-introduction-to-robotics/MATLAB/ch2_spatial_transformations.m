%% [Ch2] 공간 기술 및 좌표 변환 (Spatial Descriptions and Transformations)
% =========================================================================
% 설명:
%   본 스크립트는 3차원 공간에서 좌표계의 표현, 회전 행렬(Rotation Matrix),
%   오일러 각(Euler Angles - Z-Y-X roll-pitch-yaw), 동차 변환 행렬(Homogeneous
%   Transformation Matrix)의 기하학적 의미를 시각화하고 검증합니다.
%
% 주요 학습 내용:
%   1. 회전 행렬 R_x, R_y, R_z의 구성 및 직교성(Orthogonality, R^T * R = I, det(R)=1)
%   2. Z-Y-X 오일러 각을 통한 복합 회전 행렬 합성
%   3. 동차 변환 행렬 T (4x4)를 이용한 점의 좌표 변환 (^A P = ^A_B T * ^B P)
%   4. 3D 공간에서 변환 전/후의 좌표계(RGB 축) 시각화
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch2] 3D 공간 기술 및 좌표 변환 시뮬레이션 시작\n');
fprintf('=================================================================\n\n');

%% 1. 회전 각도 및 이동 벡터 정의
% Roll(phi, X축 회전), Pitch(theta, Y축 회전), Yaw(psi, Z축 회전)
roll_deg  = 30;   % [deg] X축 회전
pitch_deg = 45;   % [deg] Y축 회전
yaw_deg   = 60;   % [deg] Z축 회전

% 라디안 변환
phi   = deg2rad(roll_deg);
theta = deg2rad(pitch_deg);
psi   = deg2rad(yaw_deg);

% 기준 좌표계 {A}에 대한 좌표계 {B}의 원점 위치 벡터 ^A P_Borg
p_B = [1.5; 2.0; 1.0];

%% 2. 기본 회전 행렬 정의 (Elementary Rotation Matrices)
R_x = [1,        0,         0;
       0, cos(phi), -sin(phi);
       0, sin(phi),  cos(phi)];

R_y = [ cos(theta), 0, sin(theta);
                 0, 1,          0;
       -sin(theta), 0, cos(theta)];

R_z = [cos(psi), -sin(psi), 0;
       sin(psi),  cos(psi), 0;
              0,         0, 1];

% Z-Y-X 복합 회전 행렬 (^A_B R = R_z * R_y * R_x)
R_AB = R_z * R_y * R_x;

fprintf('[1] 회전 행렬 ^A_B R (Z-Y-X Euler):\n');
disp(R_AB);

% 직교성 검증: R * R^T = I, det(R) = 1
is_orthogonal = norm(R_AB * R_AB' - eye(3)) < 1e-10;
det_val = det(R_AB);
fprintf(' - 회전 행렬 직교성 검증 (R*R^T == I): %d\n', is_orthogonal);
fprintf(' - 행렬식 det(R) = %.4f (정상: 1.0000)\n\n', det_val);

%% 3. 동차 변환 행렬 구성 (^A_B T)
T_AB = [R_AB,     p_B;
        0, 0, 0,    1];

fprintf('[2] 동차 변환 행렬 ^A_B T (4x4):\n');
disp(T_AB);

%% 4. 좌표계 {B}에 정의된 점의 좌표를 {A} 좌표계로 변환
% 좌표계 {B}에서의 점 위치 ^B P
P_B = [1.0; 0.5; 0.5];
P_B_homo = [P_B; 1];

% 변환 수식: ^A P = ^A_B T * ^B P
P_A_homo = T_AB * P_B_homo;
P_A = P_A_homo(1:3);

fprintf('[3] 점의 위치 변환:\n');
fprintf(' - 좌표계 {B} 기준 점 위치 ^B P = [%.2f, %.2f, %.2f]^T\n', P_B(1), P_B(2), P_B(3));
fprintf(' - 기준 좌표계 {A} 기준 변환 ^A P = [%.2f, %.2f, %.2f]^T\n\n', P_A(1), P_A(2), P_A(3));

%% 5. 3D 시각화 (Figure 1)
figure('Name', 'Ch2: 3D Coordinate Transformations', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 100, 900, 700]);
hold on; grid on; axis equal;
view(45, 30);
xlabel('X_A [m]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Y_A [m]', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Z_A [m]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('3D Coordinate Frame Transformation\n(Roll: %d^\\circ, Pitch: %d^\\circ, Yaw: %d^\\circ)', ...
      roll_deg, pitch_deg, yaw_deg), 'FontSize', 13, 'FontWeight', 'bold');

% 축 길이 설정
axis_len = 1.0;

% (1) 기준 좌표계 {A} 그리기 (원점 [0,0,0], 단위 벡터 축)
draw_frame(eye(4), axis_len, '{A} (Base)', 'normal');

% (2) 변환된 좌표계 {B} 그리기
draw_frame(T_AB, axis_len, '{B} (Body)', 'bold');

% (3) 점 P 시각화
plot3(P_A(1), P_A(2), P_A(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
text(P_A(1)+0.1, P_A(2)+0.1, P_A(3)+0.1, 'Point P', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'm');

% {A} 원점에서 {B} 원점까지의 위치 벡터 점선
plot3([0, p_B(1)], [0, p_B(2)], [0, p_B(3)], 'k--', 'LineWidth', 1.5);
text(p_B(1)/2, p_B(2)/2, p_B(3)/2 + 0.1, '^A P_{Borg}', 'FontSize', 10, 'Color', [0.3 0.3 0.3]);

% {B} 원점에서 점 P까지의 벡터
plot3([p_B(1), P_A(1)], [p_B(2), P_A(2)], [p_B(3), P_A(3)], 'm:', 'LineWidth', 2);

xlim([-1, 3.5]); ylim([-1, 3.5]); zlim([-1, 3.0]);
hold off;

fprintf(' 시뮬레이션 및 3D 플롯 작성이 완료되었습니다.\n');

%% 보조 함수: 3차원 좌표계(RGB 축) 그리기 함수
function draw_frame(T, len, label_name, text_weight)
    origin = T(1:3, 4);
    R = T(1:3, 1:3);
    
    x_axis = R(:, 1) * len;
    y_axis = R(:, 2) * len;
    z_axis = R(:, 3) * len;
    
    % X축: 빨강 (Red), Y축: 초록 (Green), Z축: 파랑 (Blue)
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 0, ...
            'Color', [0.85 0.1 0.1], 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 0, ...
            'Color', [0.1 0.7 0.2], 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 0, ...
            'Color', [0.1 0.3 0.9], 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
        
    % 축 라벨 표기
    text(origin(1)+x_axis(1)*1.1, origin(2)+x_axis(2)*1.1, origin(3)+x_axis(3)*1.1, 'X', 'Color', [0.85 0.1 0.1], 'FontWeight', 'bold');
    text(origin(1)+y_axis(1)*1.1, origin(2)+y_axis(2)*1.1, origin(3)+y_axis(3)*1.1, 'Y', 'Color', [0.1 0.7 0.2], 'FontWeight', 'bold');
    text(origin(1)+z_axis(1)*1.1, origin(2)+z_axis(2)*1.1, origin(3)+z_axis(3)*1.1, 'Z', 'Color', [0.1 0.3 0.9], 'FontWeight', 'bold');
    
    % 좌표계 원점 이름
    text(origin(1)-0.1, origin(2)-0.1, origin(3)-0.1, label_name, 'FontSize', 11, 'FontWeight', text_weight);
end
