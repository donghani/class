# 🛠️ 로봇공학입문 MATLAB 실습실 (Robotics MATLAB Lab)

본 디렉토리는 **로봇공학입문 (Introduction to Robotics)** 수업의 핵심 이론(공간 변환, 순기구학, 역기구학, 자코비안, 동역학, 궤적 생성, 기구 설계, 선형 관절 제어, 비선형 계산 토크 제어, 힘/임피던스 제어)을 직접 시뮬레이션하고 시각화할 수 있는 MATLAB 실습 코드 모음입니다.

> 💡 **특징:** 모든 스크립트는 외부 유료 툴박스(Robotics System Toolbox 등) 없이 **순수 표준 MATLAB(기본 그래픽스, 행렬 연산, ODE 솔버)**만으로 100% 독립 실행되도록 설계되었습니다.

---

## 📂 실습 스크립트 목차 (Index)

| 파일명 | 대상 챕터 | 핵심 알고리즘 및 시각화 내용 |
| :--- | :--- | :--- |
| **[ch2_spatial_transformations.m](./ch2_spatial_transformations.m)** | 제2장 공간 기술과 변환 | 3D RGB 축 좌표계 변환, Z-Y-X 오일러 회전, 동차 변환 행렬($T$)을 이용한 점 좌표 변환 |
| **[ch3_forward_kinematics_dh.m](./ch3_forward_kinematics_dh.m)** | 제3장 순기구학 | Craig 표준 DH 파라미터 변환, 3자유도 공간 로봇 매니퓰레이터 3D 렌더링 및 엔드이펙터 계산 |
| **[ch4_inverse_kinematics.m](./ch4_inverse_kinematics.m)** | 제4장 역기구학 | 2링크 평면 로봇 기하학적 역기구학 해석, 복수 해(Elbow-Up vs Elbow-Down) 비교, 도달 가능 영역 판별 |
| **[ch5_jacobian_and_velocity.m](./ch5_jacobian_and_velocity.m)** | 제5장 자코비안 & 정적 힘 | 미분 기구학 자코비안($J$), 특이점(Singularity) 분석, Yoshikawa 가조작성 속도 타원체(Velocity Ellipsoid) 작도 |
| **[ch6_manipulator_dynamics.m](./ch6_manipulator_dynamics.m)** | 제6장 동역학 | 2자유도 로봇의 Euler-Lagrange 비선형 동역학($M, C, G$), `ode45` 자유 낙하 진자 시뮬레이션, 역학적 에너지 보존 검증 |
| **[ch7_trajectory_generation.m](./ch7_trajectory_generation.m)** | 제7장 궤적 생성 | 3차 다항식(Cubic), 5차 다항식(Quintic), 사다리꼴 속도(LSPB) 궤적 비교 ($q, \dot{q}, \ddot{q}, \text{Jerk}$) |
| **[ch8_mechanism_design_sizing.m](./ch8_mechanism_design_sizing.m)** | 제8장 기구 설계 | 모터-감속기 관성 매칭 최적 감속비($N_{opt}=\sqrt{J_L/J_m}$), 요구 토크 곡선, 링크 길이 비율에 따른 작업 영역 최적화 |
| **[ch9_linear_joint_control.m](./ch9_linear_joint_control.m)** | 제9장 선형 제어 | 독립 관절 P, PD, PID 제어기 비교, 중력 외란 하 정상상태 오차(Droop), 중력 피드포워드(Gravity Feedforward) 결합 |
| **[ch10_computed_torque_control.m](./ch10_computed_torque_control.m)** | 제10장 비선형 제어 | 계산 토크 제어(Computed Torque Control / Feedback Linearization)를 통한 고속 궤적 추종 및 모델 불확실성 강인성 평가 |
| **[ch11_force_and_impedance_control.m](./ch11_force_and_impedance_control.m)** | 제11장 힘 제어 | 강체 벽($K_{env}$) 접촉 환경에서의 임피던스 제어(Impedance Control) 및 하이브리드 힘/위치 제어 시뮬레이션 |
| **[validate_all.m](./validate_all.m)** | 전체 검증 | 10개 실습 스크립트의 전체 배치 자동 검증 러너 |

---

## 🔬 챕터별 상세 설명 및 학습 가이드

### 1. `ch2_spatial_transformations.m` (공간 기술 및 좌표 변환)
- **수학적 배경**: 
  $$\mathbf{R}_{ZYX}(\alpha, \beta, \gamma) = \mathbf{R}_z(\alpha) \mathbf{R}_y(\beta) \mathbf{R}_x(\gamma)$$
  $$^A \mathbf{P} = {^A_B \mathbf{T}} \cdot {^B \mathbf{P}} = \begin{bmatrix} ^A_B \mathbf{R} & ^A \mathbf{P}_{Borg} \\ \mathbf{0}^T & 1 \end{bmatrix} \begin{bmatrix} ^B \mathbf{P} \\ 1 \end{bmatrix}$$
- **실습 포인트**: Roll-Pitch-Yaw 각도 변경 시 3차원 공간에서 좌표계의 3색 축(Red: X, Green: Y, Blue: Z)이 어떻게 회전하고 평행이동하는지 시각적으로 관찰합니다.

### 2. `ch3_forward_kinematics_dh.m` (순기구학 및 DH 파라미터)
- **수학적 배경**: Craig 표기법에 따른 인접 링크 간 동차 변환 행렬
  $$^{i-1}_i \mathbf{T} = \text{Rot}_X(\alpha_{i-1}) \text{Trans}_X(a_{i-1}) \text{Rot}_Z(\theta_i) \text{Trans}_Z(d_i)$$
- **실습 포인트**: 3자유도 로봇 팔의 관절 각도 $[\theta_1, \theta_2, \theta_3]$를 변경하면서 3D 공간 상에서 로봇 팔의 포즈 변화와 엔드이펙터의 최종 3D 좌표를 확인합니다.

### 3. `ch4_inverse_kinematics.m` (역기구학 해석 및 다중 해)
- **수학적 배경**: 코사인 제2법칙을 이용한 관절각 해석적 계산
  $$\cos\theta_2 = \frac{x_d^2 + y_d^2 - l_1^2 - l_2^2}{2 l_1 l_2}, \quad \theta_1 = \text{atan2}(y_d, x_d) - \text{atan2}(l_2\sin\theta_2, l_1 + l_2\cos\theta_2)$$
- **실습 포인트**: 동일한 목표 위치 $(x_d, y_d)$에 도달하기 위한 **Elbow-Up**과 **Elbow-Down** 두 가지 자세를 동시에 계산하고, 작업 공간 경계 초과 시의 예외 처리를 확인합니다.

### 4. `ch5_jacobian_and_velocity.m` (자코비안 및 속도 타원체)
- **수학적 배경**: 엔드이펙터 속도와 관절 속도의 선형 사상 $\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$, Yoshikawa 가조작성 지수 $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$
- **실습 포인트**: 일반 포즈, 특이점 근접 포즈, 완전 특이점(팔이 180도 완전히 펴진 상태)에서의 속도 타원체가 찌그러지는 형태를 비교합니다.

### 5. `ch6_manipulator_dynamics.m` (Euler-Lagrange 동역학)
- **수학적 배경**: 
  $$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau}$$
- **실습 포인트**: 토크가 0인 상태에서 중력에 의해 요동치는 2링크 로봇의 비선형 진자 운동을 `ode45`로 적분하고, 운동에너지와 위치에너지의 합(총 기계적 에너지)이 일정하게 보존됨을 확인합니다.

### 6. `ch7_trajectory_generation.m` (궤적 생성 및 저크 비교)
- **실습 포인트**: 3차 다항식(Cubic), 5차 다항식(Quintic), 사다리꼴 속도(LSPB)의 가속도 불연속성 및 저크(Jerk) 프로파일을 대조하여, 로봇의 진동을 방지하는 부드러운 모션 프로파일 설계 기법을 배웁니다.

### 7. `ch8_mechanism_design_sizing.m` (기구 설계 및 모터 사이징)
- **수학적 배경**: 최적 관성 매칭 감속비 $N_{opt} = \sqrt{J_L / J_m}$
- **실습 포인트**: 감속비 $N$에 따른 모터 요구 토크 곡선과 가속도 효율 곡선의 정점을 파악하고, 링크 길이 비율 $l_2 / (l_1 + l_2)$에 따른 작업 공간 면적 최적화($l_1 = l_2$일 때 최대)를 분석합니다.

### 8. `ch9_linear_joint_control.m` (선형 관절 제어 및 중력 보상)
- **실습 포인트**: P 제어와 PD 제어에서 발생하는 중력 드룹(정상상태 오차)을 확인하고, PID 제어기의 적분 동작 및 **PD + Gravity Feedforward** 제어기가 오차 없이 신속하게 목표 각도에 수렴하는 과정을 비교합니다.

### 9. `ch10_computed_torque_control.m` (계산 토크 비선형 제어)
- **수학적 배경**: $\boldsymbol{\tau} = \hat{\mathbf{M}}(\mathbf{q})(\ddot{\mathbf{q}}_d + \mathbf{K}_v \dot{\mathbf{e}} + \mathbf{K}_p \mathbf{e}) + \hat{\mathbf{C}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \hat{\mathbf{G}}(\mathbf{q})$
- **실습 포인트**: 고속 정현파 궤적 추종 시 단순 PD 제어 대비 Computed Torque 제어의 비선형 동역학 상쇄 효과와 궤적 추종 정밀도(오차 대폭 감소)를 확인합니다.

### 10. `ch11_force_and_impedance_control.m` (힘 및 임피던스 제어)
- **수학적 배경**: $M_d(\ddot{x} - \ddot{x}_d) + B_d(\dot{x} - \dot{x}_d) + K_d(x - x_d) = -F_{ext}$
- **실습 포인트**: 단단한 벽($K_{env} = 5000\text{ N/m}$)과 접촉할 때 엔드이펙터가 파손되지 않고 부드럽게 감속하며 목표 접촉력으로 수렴하는 물리적 거동을 관찰합니다.

---

## 🚀 빠른 실행 (How to Run)

1. **MATLAB**을 실행합니다.
2. 작업 디렉토리를 `class/class-introduction-to-robotics/MATLAB`으로 이동합니다.
3. 원하는 스크립트를 열고 `F5`를 누르거나, 명령창에서 파일명을 입력합니다:
   ```matlab
   ch3_forward_kinematics_dh
   ```
4. 모든 스크립트를 한 번에 일괄 테스트하려면:
   ```matlab
   validate_all
   ```
