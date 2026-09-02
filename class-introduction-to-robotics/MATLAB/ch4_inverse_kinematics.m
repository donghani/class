%% [Ch4] 역기구학 해석 (Inverse Kinematics & Multiple Solutions)
% =========================================================================
% 설명:
%   본 스크립트는 2자유도/3자유도 평면 및 공간 로봇에 대한 기하학적/대수학적
%   역기구학(Inverse Kinematics) 해석 솔루션을 제공합니다.
%
% 주요 학습 내용:
%   1. 코사인 법칙을 활용한 기하학적 역기구학 해법 유도
%   2. 동일한 목표 엔드이펙터 위치에 대한 복수 해(Elbow-Up vs Elbow-Down) 계산
%   3. 도달 가능 작업 영역(Reachable Workspace) 판별 및 특이점(Singularity) 확인
%   4. 두 가지 해의 로봇 포즈 2D/3D 비교 시각화
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch4] 역기구학(Inverse Kinematics) 해석 및 다중 해 비교\n');
fprintf('=================================================================\n\n');

%% 1. 로봇 링크 파라미터 정의 (2-Link Planar Arm)
l1 = 1.0;   % 링크 1 길이 [m]
l2 = 0.7;   % 링크 2 길이 [m]

% 목표 엔드이펙터 위치 설정 (x_d, y_d)
x_d = 1.2;
y_d = 0.8;

fprintf('[1] 로봇 파라미터 및 목표 지점:\n');
fprintf(' - 링크 길이: l1 = %.2f m, l2 = %.2f m\n', l1, l2);
fprintf(' - 목표 위치: (x_d, y_d) = (%.2f, %.2f) m\n\n', x_d, y_d);

%% 2. 작업 공간(Workspace) 판별
r_target = sqrt(x_d^2 + y_d^2);
r_max = l1 + l2;
r_min = abs(l1 - l2);

if r_target > r_max || r_target < r_min
    error('목표 위치가 로봇의 작업 공간(Workspace)을 벗어났습니다! (도달 불가)');
else
    fprintf('[2] 작업 공간 검증 완료 (도달 가능 영역): %.2f m <= %.2f m <= %.2f m\n\n', ...
            r_min, r_target, r_max);
end

%% 3. 기하학적 역기구학 계산 (Geometric IK)
% 코사인 제2법칙: r^2 = l1^2 + l2^2 - 2*l1*l2*cos(pi - theta2)
%                = l1^2 + l2^2 + 2*l1*l2*cos(theta2)
cos_th2 = (x_d^2 + y_d^2 - l1^2 - l2^2) / (2 * l1 * l2);

% 수치 오차 클램핑 (-1 ~ 1)
cos_th2 = max(min(cos_th2, 1), -1);

% (1) Solution 1: Elbow-Down (theta2 > 0)
sin_th2_down = sqrt(1 - cos_th2^2);
th2_down = atan2(sin_th2_down, cos_th2);
k1 = l1 + l2 * cos(th2_down);
k2 = l2 * sin(th2_down);
th1_down = atan2(y_d, x_d) - atan2(k2, k1);

% (2) Solution 2: Elbow-Up (theta2 < 0)
sin_th2_up = -sqrt(1 - cos_th2^2);
th2_up = atan2(sin_th2_up, cos_th2);
k1_up = l1 + l2 * cos(th2_up);
k2_up = l2 * sin(th2_up);
th1_up = atan2(y_d, x_d) - atan2(k2_up, k1_up);

fprintf('[3] 역기구학 해석 결과:\n');
fprintf(' [Sol 1: Elbow-Down] theta1 = %6.2f deg, theta2 = %6.2f deg\n', ...
        rad2deg(th1_down), rad2deg(th2_down));
fprintf(' [Sol 2: Elbow-Up]   theta1 = %6.2f deg, theta2 = %6.2f deg\n\n', ...
        rad2deg(th1_up), rad2deg(th2_up));

%% 4. 순기구학을 통한 검증 (Forward Kinematics Verification)
% Sol 1 검증
x_calc_down = l1*cos(th1_down) + l2*cos(th1_down + th2_down);
y_calc_down = l1*sin(th1_down) + l2*sin(th1_down + th2_down);
err_down = norm([x_d - x_calc_down; y_d - y_calc_down]);

% Sol 2 검증
x_calc_up = l1*cos(th1_up) + l2*cos(th1_up + th2_up);
y_calc_up = l1*sin(th1_up) + l2*sin(th1_up + th2_up);
err_up = norm([x_d - x_calc_up; y_d - y_calc_up]);

fprintf('[4] 순기구학 재검증 오차:\n');
fprintf(' - Sol 1 위치 오차: %.2e m\n', err_down);
fprintf(' - Sol 2 위치 오차: %.2e m\n\n', err_up);

%% 5. 시각화 (Figure 1: Elbow-Up vs Elbow-Down 비교)
figure('Name', 'Ch4: Inverse Kinematics Solutions', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [200, 100, 950, 700]);
hold on; grid on; axis equal;
xlabel('X [m]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('2-Link Robot Inverse Kinematics Solutions\nTarget: (%.2f, %.2f) m', x_d, y_d), ...
      'FontSize', 13, 'FontWeight', 'bold');

% 작업 공간 경계 작도 (원형 궤적)
th_circle = linspace(0, 2*pi, 100);
plot(r_max*cos(th_circle), r_max*sin(th_circle), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Max Reach Boundary');
if r_min > 0
    plot(r_min*cos(th_circle), r_min*sin(th_circle), 'k:', 'LineWidth', 1.2, 'DisplayName', 'Min Reach Boundary');
end

% Sol 1: Elbow-Down 관절 위치
p0 = [0, 0];
p1_down = [l1*cos(th1_down), l1*sin(th1_down)];
p2_down = [x_calc_down, y_calc_down];

% Sol 2: Elbow-Up 관절 위치
p1_up = [l1*cos(th1_up), l1*sin(th1_up)];
p2_up = [x_calc_up, y_calc_up];

% Sol 1 (Elbow-Down) 로봇 팔 플롯
h1 = plot([p0(1), p1_down(1), p2_down(1)], [p0(2), p1_down(2), p2_down(2)], ...
          '-o', 'LineWidth', 4, 'Color', [0.15 0.55 0.9], 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.1 0.3 0.8], 'DisplayName', 'Solution 1 (Elbow-Down)');

% Sol 2 (Elbow-Up) 로봇 팔 플롯
h2 = plot([p0(1), p1_up(1), p2_up(1)], [p0(2), p1_up(2), p2_up(2)], ...
          '-^', 'LineWidth', 4, 'Color', [0.9 0.35 0.15], 'MarkerSize', 8, ...
          'MarkerFaceColor', [0.8 0.2 0.1], 'DisplayName', 'Solution 2 (Elbow-Up)');

% 목표 위치 마커
plot(x_d, y_d, 'kp', 'MarkerSize', 16, 'MarkerFaceColor', 'y', 'LineWidth', 1.5, ...
     'DisplayName', 'Target Position (x_d, y_d)');

legend([h1, h2], 'Location', 'northwest', 'FontSize', 10);
xlim([-1.8, 2.0]); ylim([-1.8, 2.0]);
hold off;

fprintf(' 역기구학 해석 및 2D 시각화가 완료되었습니다.\n');
