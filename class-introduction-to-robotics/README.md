# 🤖 로봇공학입문 (Introduction to Robotics)
> **경희대학교 전자공학과 김동한 교수님 강의 자료실**  
> 본 저장소는 로봇공학입문(Introduction to Robotics) 수업의 강의 자료(PDF) 및 로봇 기구학·동역학·제어를 체계적으로 실습할 수 있는 MATLAB 예제 패키지를 정리해놓은 공간입니다.

---

## 📢 과목 및 강사 정보 (Course Information)

| 구분 | 정보 및 링크 |
| :--- | :--- |
| **👨‍🏫 담당 교수** | **김동한 교수** (전자공학과, 전자정보대학 609호, 내선 3831) |
| **✉️ 이메일** | [donghani@khu.ac.kr](mailto:donghani@khu.ac.kr) |
| **⏱️ 상담 시간** | 매주 수업 종료 후 1시간 |
| **📖 주교재** | **Introduction to Robotics: Mechanics and Control, 4th Edition** (John J. Craig 저) |
| **🔗 강의 영상** | [YouTube 채널 (경희대 김동한)](https://www.youtube.com/channel/UCT_h-5YhlC0t9LEdVckdrXQ) |
| **💻 실습 포털** | [경희대학교 e-Campus](https://e-campus.khu.ac.kr) |

---

## 📚 주차별 강의 자료 (Lecture Slides)

아래 강의 자료(PDF)는 본 저장소의 루트 폴더에 정리되어 있어 다운로드하여 확인할 수 있습니다.

| 장 / 주제 | 파일명 (다운로드 링크) | 주요 학습 내용 |
| :--- | :--- | :--- |
| **과목 소개** | [Introduction to Robotics Ch0.pdf](./Introduction%20to%20Robotics%20Ch0.pdf) | 과목 개요, 로봇공학의 발전 흐름 및 전체 강의 일정 |
| **제1장** | [Introduction to Robotics Ch1.pdf](./Introduction%20to%20Robotics%20Ch1.pdf) | 로봇의 정의, 분류, 산업용 및 서비스 매니퓰레이터 기초 |
| **제2장** | [Introduction to Robotics Ch2.pdf](./Introduction%20to%20Robotics%20Ch2.pdf) | 공간 기술 및 좌표 변환 (회전 행렬, 오일러 각, 동차 변환 행렬 $T$) |
| **제3장** | [Introduction to Robotics Ch3.pdf](./Introduction%20to%20Robotics%20Ch3.pdf) | 매니퓰레이터 순기구학 (Denavit-Hartenberg 파라미터 링크 모델링) |
| **제4장** | [Introduction to Robotics Ch4.pdf](./Introduction%20to%20Robotics%20Ch4.pdf) | 매니퓰레이터 역기구학 (해석적 해법, Pieper 해법, 다중 해) |
| **제5장** | [Introduction to Robotics Ch5.pdf](./Introduction%20to%20Robotics%20Ch5.pdf) | 자코비안, 미분 기구학 및 정적 힘 전달 ($\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$), 특이점 |
| **제6장** | [Introduction to Robotics Ch6.pdf](./Introduction%20to%20Robotics%20Ch6.pdf) | 매니퓰레이터 동역학 (Newton-Euler 및 Euler-Lagrange 방정식) |
| **제7장** | [Introduction to Robotics Ch7.pdf](./Introduction%20to%20Robotics%20Ch7.pdf) | 궤적 생성 및 경로 계획 (Cubic, Quintic, 사다리꼴 속도 프로파일) |
| **제8장** | [Introduction to Robotics Ch8.pdf](./Introduction%20to%20Robotics%20Ch8.pdf) | 매니퓰레이터 기구 설계 (모터/감속기 사이징 및 관성 매칭 최적화) |
| **제9장** | [Introduction to Robotics Ch9.pdf](./Introduction%20to%20Robotics%20Ch9.pdf) | 매니퓰레이터의 선형 제어 (독립 관절 PID 제어, 중력 피드포워드) |
| **제10장** | [Introduction to Robotics Ch10.pdf](./Introduction%20to%20Robotics%20Ch10.pdf) | 매니퓰레이터의 비선형 제어 (계산 토크 제어 Computed Torque Control) |
| **제11장** | [Introduction to Robotics Ch11.pdf](./Introduction%20to%20Robotics%20Ch11.pdf) | 매니퓰레이터의 힘 제어 (강성 환경 접촉, 임피던스 제어, 하이브리드 제어) |

---

## 🛠️ MATLAB 로봇공학 실습실 (`/MATLAB`)

로봇 기구학, 동역학, 궤적 생성, 제어 이론을 시각적이고 직관적으로 학습할 수 있는 종합 실습 패키지입니다.  
별도의 외부 툴박스 설치 없이 **순수 표준 MATLAB 기본 기능만으로 100% 자립 실행**됩니다.

> 💡 **상세한 실습 가이드 및 수학적 유도는 [MATLAB 종합 안내서](./MATLAB/README.md)를 참고하세요.**

### 📐 1. 기구학 및 변환 (Kinematics & Transformations)
* **[ch2_spatial_transformations.m](./MATLAB/ch2_spatial_transformations.m):** 3D 공간에서 3색 RGB 좌표축 회전(Z-Y-X 오일러 각) 및 동차 변환 행렬($T$)을 통한 점의 위치 변환 시각화.
* **[ch3_forward_kinematics_dh.m](./MATLAB/ch3_forward_kinematics_dh.m):** Craig 표준 DH 파라미터 테이블을 기반으로 3자유도 공간 로봇의 순기구학 계산 및 3D 로봇 모델 렌더링.
* **[ch4_inverse_kinematics.m](./MATLAB/ch4_inverse_kinematics.m):** 목표 엔드이펙터 위치에 대한 기하학적 역기구학 계산, **Elbow-Up vs Elbow-Down** 다중 해 비교 및 작업 영역 판별.
* **[ch5_jacobian_and_velocity.m](./MATLAB/ch5_jacobian_and_velocity.m):** 자코비안($J$) 계산, 특이점(Singularity) 분석, Yoshikawa 가조작성 타원체(Velocity Ellipsoid) 작도.

### ⚙️ 2. 동역학 및 기구 설계 (Dynamics & Mechanism Design)
* **[ch6_manipulator_dynamics.m](./MATLAB/ch6_manipulator_dynamics.m):** 2자유도 로봇의 Euler-Lagrange 비선형 미분방정식($M, C, G$)을 `ode45`로 적분하여 자유 진자 운동 시뮬레이션 및 역학적 에너지 보존 법칙 검증.
* **[ch7_trajectory_generation.m](./MATLAB/ch7_trajectory_generation.m):** 3차(Cubic), 5차(Quintic), 사다리꼴 속도(LSPB) 궤적의 위치, 속도, 가속도 및 저크(Jerk) 프로파일 종합 대조.
* **[ch8_mechanism_design_sizing.m](./MATLAB/ch8_mechanism_design_sizing.m):** 감속비($N$)에 따른 관성 매칭 최적화($N_{opt}=\sqrt{J_L/J_m}$), 요구 토크 사이징 곡선 및 링크 길이 비율별 작업 면적 최적화.

### 🎛️ 3. 로봇 제어 (Robot Motion & Force Control)
* **[ch9_linear_joint_control.m](./MATLAB/ch9_linear_joint_control.m):** 독립 관절 P, PD, PID 제어기 비교, 중력 외란에 의한 오차 분석 및 중력 피드포워드(Gravity Feedforward) 결합을 통한 무오차 제어.
* **[ch10_computed_torque_control.m](./MATLAB/ch10_computed_torque_control.m):** 역동역학을 이용한 비선형 계산 토크 제어(Computed Torque Control)로 고속 궤적 정밀 추종 및 모델 불확실성 강인성 평가.
* **[ch11_force_and_impedance_control.m](./MATLAB/ch11_force_and_impedance_control.m):** 강성 벽($K_{env}$) 접촉 시 충격을 완화하고 안정적인 접촉력을 유지하는 임피던스 제어 및 하이브리드 힘/위치 제어 시뮬레이션.

---

## 🚀 시작하기 (Quick Start)

### 1. 로컬 저장소 내려받기 (Sparse Checkout 적용)
로봇공학 실습 파일과 강의 자료만 가볍게 다운로드하고 싶은 경우 아래 명령어를 실행하세요.

```bash
# 저장소 복제 (기본)
git clone https://github.com/donghani/class.git
cd class/class-introduction-to-robotics
```

### 2. MATLAB 실행 및 실습
1. **MATLAB**을 실행하고 `class-introduction-to-robotics/MATLAB` 폴더를 현재 폴더로 설정합니다.
2. 원하는 스크립트(예: `ch3_forward_kinematics_dh.m`)를 열고 실행(`F5`)합니다.
3. 모든 예제를 한 번에 무결성 검증하려면 명령창에 다음을 입력합니다:
   ```matlab
   validate_all
   ```
