%% [Ch5] 자코비안, 속도 및 정적 힘 (Jacobians, Velocities and Static Forces)
% =========================================================================
% 설명:
%   본 스크립트는 매니퓰레이터의 자코비안(Jacobian) 행렬을 유도하고,
%   관절 속도와 엔드이펙터 속도 사이의 미분 기구학 관계, 정적 힘 전달(tau = J^T * F),
%   기구학적 특이점(Singularity) 및 가조작성 타원체(Manipulability Ellipsoid)를 작도합니다.
%
% 주요 학습 내용:
%   1. 2자유도 평면 로봇의 기하학적 자코비안 J(q) 계산 (v = J * q_dot)
%   2. 정적 힘 전달 관계 (tau = J^T * F) 및 가상 일의 원리
%   3. 특이점 판별 (det(J) = 0 또는 cond(J) -> inf)
%   4. Yoshikawa 가조작성 지수 w = sqrt(det(J * J^T)) 계산
%   5. 엔드이펙터에서의 속도 타원체(Velocity Ellipsoid) 시각화
% =========================================================================

clc; clear; close all;

fprintf('=================================================================\n');
fprintf('  [Ch5] 자코비안 및 가조작성 속도 타원체 시뮬레이션\n');
fprintf('=================================================================\n\n');

%% 1. 로봇 링크 파라미터 정의
l1 = 1.0;   % [m]
l2 = 0.8;   % [m]

% 세 가지 대표 자세에 대한 관절 각도 설정
% Case A: 일반적인 자세 (높은 가조작성)
% Case B: 특이점에 가까운 자세 (팔이 거의 펴짐)
% Case C: 완전한 특이점 (theta2 = 0, 완전히 펴짐)
cases = {
    'Case A: Dexterous Pose',    [deg2rad(30), deg2rad(60)];
    'Case B: Near Singular Pose', [deg2rad(15), deg2rad(15)];
    'Case C: Fully Extended (Singular)', [deg2rad(10), 0]
};

figure('Name', 'Ch5: Jacobians & Manipulability Ellipsoids', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 100, 1200, 450]);

for c_idx = 1:3
    case_name = cases{c_idx, 1};
    q = cases{c_idx, 2};
    th1 = q(1);
    th2 = q(2);
    
    %% 2. 순기구학 및 자코비안 계산
    x_ee = l1*cos(th1) + l2*cos(th1 + th2);
    y_ee = l1*sin(th1) + l2*sin(th1 + th2);
    
    % 자코비안 행렬: J = [dx/dth1, dx/dth2; dy/dth1, dy/dth2]
    J = [
        -l1*sin(th1) - l2*sin(th1 + th2),  -l2*sin(th1 + th2);
         l1*cos(th1) + l2*cos(th1 + th2),   l2*cos(th1 + th2)
    ];

    % 자코비안 행렬식: det(J) = l1 * l2 * sin(th2)
    det_J = det(J);
    
    % Yoshikawa 가조작성 지수 w = sqrt(det(J * J^T))
    w_manip = sqrt(det(J * J'));
    
    fprintf('--- [%s] ---\n', case_name);
    fprintf(' 관절 각도: th1 = %.1f deg, th2 = %.1f deg\n', rad2deg(th1), rad2deg(th2));
    fprintf(' 엔드이펙터 위치: (x, y) = (%.3f, %.3f) m\n', x_ee, y_ee);
    fprintf(' 자코비안 행렬 J(q):\n');
    disp(J);
    fprintf(' det(J) = %.4f | 가조작성 지수 w = %.4f\n\n', det_J, w_manip);

    %% 3. 속도 타원체 (Velocity Ellipsoid) 계산
    % ||q_dot|| <= 1 인 단위 관절 속도 구에 대응하는 엔드이펙터 속도: v^T * (J * J^T)^-1 * v <= 1
    % SVD (Singular Value Decomposition): J = U * S * V^T
    [U, S, ~] = svd(J);
    sigma1 = S(1, 1);
    sigma2 = S(2, 2);
    
    % 타원체 작도 점 생성
    phi_t = linspace(0, 2*pi, 100);
    circle_pts = [cos(phi_t); sin(phi_t)];
    % 속도 타원 점들: v_ellipse = J * circle_pts
    v_ellipse = J * circle_pts * 0.4; % 보기 좋게 스케일 조정 (0.4)
    
    %% 4. 서브플롯 시각화
    subplot(1, 3, c_idx);
    hold on; grid on; axis equal;
    
    % 로봇 링크 위치
    p0 = [0, 0];
    p1 = [l1*cos(th1), l1*sin(th1)];
    p2 = [x_ee, y_ee];
    
    % 링크 작도
    plot([p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], '-o', ...
         'LineWidth', 4, 'Color', [0.2 0.45 0.85], 'MarkerSize', 8, ...
         'MarkerFaceColor', [0.9 0.5 0.1]);
     
    % 속도 타원체 작도 (엔드이펙터 중심)
    ellipse_x = x_ee + v_ellipse(1, :);
    ellipse_y = y_ee + v_ellipse(2, :);
    patch(ellipse_x, ellipse_y, [0.2 0.8 0.4], 'FaceAlpha', 0.35, 'EdgeColor', [0.1 0.6 0.2], 'LineWidth', 1.5);
    
    % 주축 방향 화살표 (U1, U2 방향 벡터)
    quiver(x_ee, y_ee, U(1,1)*sigma1*0.4, U(2,1)*sigma1*0.4, 0, 'Color', [0.8 0.1 0.1], 'LineWidth', 2);
    quiver(x_ee, y_ee, U(1,2)*sigma2*0.4, U(2,2)*sigma2*0.4, 0, 'Color', [0.1 0.2 0.8], 'LineWidth', 2);
    
    title(sprintf('%s\n\\mu = %.3f, det(J) = %.3f', case_name, w_manip, det_J), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]', 'FontWeight', 'bold'); ylabel('Y [m]', 'FontWeight', 'bold');
    xlim([-0.2, 2.2]); ylim([-0.2, 2.0]);
    hold off;
end

fprintf(' 자코비안 및 가조작성 타원체 시각화 완료.\n');
