# 🤖 두산 로봇 실시간 모니터링 및 원격 제어 시스템

## 📋 프로젝트 개요

두산 협동로봇(M0609)을 **로컬 네트워크**와 **인터넷 어디서나** 모니터링하고 제어할 수 있는 웹 기반 시스템입니다.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           시스템 아키텍처                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   [두산 로봇]                                                                │
│       │                                                                     │
│       │ ROS2 Topics/Services                                                │
│       ▼                                                                     │
│   ┌─────────────────┐         ┌──────────────────────┐                      │
│   │   app_real.py   │◄───────►│  Firebase Realtime   │                      │
│   │  (Flask+ROS2)   │         │     Database         │                      │
│   └────────┬────────┘         └──────────┬───────────┘                      │
│            │                             │                                  │
│            │ SocketIO                    │ Firebase SDK                     │
│            ▼                             ▼                                  │
│   ┌─────────────────┐         ┌──────────────────────┐                      │
│   │    로컬 웹      │         │     외부 웹          │                      │
│   │ localhost:5000  │         │ rokey-c-2.web.app    │                      │
│   │  (고속/전기능)   │         │ (인터넷 어디서나)     │                      │
│   └─────────────────┘         └──────────────────────┘                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🌐 두 가지 웹 인터페이스

### 1. 로컬 웹 (http://localhost:5000)
- **연결 방식**: SocketIO (직접 연결)
- **응답 속도**: ~10ms (매우 빠름)
- **기능**: 전체 기능 (조인트, TCP, I/O, 3D 뷰어, 진자 테스트 등)
- **접근**: 같은 네트워크 내에서만

### 2. 외부 웹 (https://rokey-c-2.web.app)
- **연결 방식**: Firebase Realtime Database 경유
- **응답 속도**: ~500ms (인터넷 경유)
- **기능**: 핵심 제어 (그리퍼, 홈, 속도, 비상정지)
- **접근**: 인터넷 어디서나 (스마트폰 포함)

---

## 🔥 Firebase Realtime Database의 역할

### Firebase란?
Google에서 제공하는 **클라우드 데이터베이스** 서비스입니다.
데이터가 변경되면 **실시간으로** 모든 연결된 클라이언트에 전달됩니다.

### 왜 필요한가?
```
문제: 외부에서 로봇에 직접 접근 불가능
      (로봇은 로컬 네트워크 안에 있음)

해결: Firebase를 "중계 서버"로 사용
      로봇 ←→ Firebase ←→ 외부 사용자
```

### 데이터 흐름

#### 📤 모니터링 (로봇 → 외부)
```
[로봇] → [app_real.py] → [Firebase DB] → [외부 웹페이지]
         (1초마다 업로드)   (클라우드)    (실시간 수신)
```

#### 📥 제어 (외부 → 로봇)
```
[외부 웹페이지] → [Firebase DB] → [app_real.py] → [로봇]
   (명령 전송)     (명령 저장)    (리스너가 수신)  (실행)
```

### 저장되는 데이터 구조
```
Firebase Database (rokey-c-2-default-rtdb)
│
├── /robot_status          ← 로봇 상태 (1초마다 업데이트)
│   ├── connected: true
│   ├── joint_position: [0, 0, 90, 0, 90, 0]
│   ├── tcp_position: [278.9, 4.7, 214.8, ...]
│   ├── operation_speed: 100
│   ├── robot_state: 1 (STANDBY)
│   ├── robot_mode: 1 (AUTONOMOUS)
│   ├── access_control: 1 (제어권 보유)
│   └── gripper: {do1: 0, do2: 1}
│
└── /robot_commands        ← 제어 명령 (외부에서 전송)
    ├── command: "gripper_open"
    ├── value: null
    └── processed: 1234567890 (처리 완료 시간)
```

---

## ✨ 주요 기능

### 🖥️ 로컬 웹 기능
| 기능 | 설명 |
|------|------|
| 조인트 모니터링 | 6축 각도, 속도, 토크, 온도 실시간 표시 |
| TCP 모니터링 | X, Y, Z, A, B, C 위치 및 외력 표시 |
| I/O 제어 | 디지털 I/O 16채널 모니터링 및 제어 (1~16번) |
| 그리퍼 제어 | 열기/닫기 버튼 |
| 홈 이동 | 사용자 홈 [0,0,90,0,90,0] / 기계적 홈 [0,0,0,0,0,0] |
| 속도 제어 | 1~100% 슬라이더 (50ms 반응) + 100% 리셋 버튼 |
| 진자 테스트 | 선택한 축으로 왕복 운동 테스트 |
| 3D 로봇 뷰어 | 실시간 로봇 자세 3D 시각화 |
| 비상 정지 | 즉시 정지 |

### 📱 외부 웹 기능
| 기능 | 설명 |
|------|------|
| 상태 모니터링 | 조인트, TCP, 로봇 상태, 제어권 |
| 그리퍼 제어 | 열기/닫기 |
| 홈 이동 | 사용자 홈 / 기계적 홈 |
| 속도 제어 | 1~100% + 100% 리셋 버튼 |
| 비상 정지 | 즉시 정지 |
| 명령 로그 | 전송한 명령 기록 |

---

## 🚀 실행 방법

### 사전 요구사항
- Ubuntu 22.04
- ROS2 Humble
- 두산 로봇 드라이버 (dsr_bringup2)
- Python 3.10+
- firebase-admin 패키지

### Step 1: 로봇 드라이버 실행
```bash
# 터미널 1: 두산 로봇 드라이버 실행
cd ~/cobot1_ws
source install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

### Step 2: 모니터링 서버 실행
```bash
# 터미널 2: 웹 서버 실행
cd ~/cobot1_ws/src/logistics_monitor
source ~/cobot1_ws/install/setup.bash
python3 app_real.py
```

실행 시 다음 메시지가 나오면 성공:
```
[INFO] Logistics Monitor Node Started
[INFO] Subscribed to /dsr01/joint_states
✅ Firebase 연동 활성화
🔥 Firebase 업로드 스레드 시작
🔥 Firebase 명령 리스너 시작
 * Running on http://0.0.0.0:5000
```

### Step 3: 웹 접속

| 접속 방법 | URL | 설명 |
|----------|-----|------|
| 로컬 PC | http://localhost:5000 | 전체 기능 |
| 같은 네트워크 PC | http://[PC IP]:5000 | 전체 기능 |
| 외부 (인터넷) | https://rokey-c-2.web.app | 핵심 제어 |

---

## 📁 프로젝트 파일 구조

```
logistics_monitor/
├── app_real.py                 # 메인 서버 (Flask + ROS2 + Firebase)
├── templates/
│   ├── index_full.html         # 로컬 웹 (전체 기능)
│   └── index_external.html     # 외부 웹 템플릿
├── config/
│   ├── serviceAccountKey.json  # Firebase 서버 인증 키 (비공개)
│   └── firebase_web_config.json # Firebase 웹 설정
└── firebase_hosting/
    ├── firebase.json           # Firebase Hosting 설정
    ├── .firebaserc             # 프로젝트 연결 설정
    └── public/
        └── index.html          # 배포된 외부 웹페이지
```

---

## 🔧 기술 스택

| 구분 | 기술 |
|------|------|
| 로봇 통신 | ROS2 Humble, dsr_msgs2 |
| 백엔드 | Python, Flask, Flask-SocketIO |
| 프론트엔드 | HTML5, CSS3, JavaScript |
| 실시간 통신 (로컬) | Socket.IO |
| 클라우드 DB | Firebase Realtime Database |
| 외부 호스팅 | Firebase Hosting |
| 3D 시각화 | Three.js |

---

## ⚠️ 주의사항

1. **제어권 확인**: 외부에서 제어 전 "제어권: ✅ 보유 중" 확인 필요
2. **속도 변경**: 동작 중 속도 변경은 **다음 동작부터** 적용됨 (로봇 특성)
3. **비상 정지**: 언제든 비상 정지 버튼으로 로봇 즉시 정지 가능
4. **Firebase 보안**: serviceAccountKey.json은 절대 공개 저장소에 업로드 금지

---

## 🔄 외부 웹 업데이트 방법

외부 웹 수정 후 배포:
```bash
# 템플릿 복사 후 Firebase에 배포
cp templates/index_external.html firebase_hosting/public/index.html
cd firebase_hosting
firebase deploy --only hosting
```

---

## 👥 개발팀

**ROKEY C조**

---

## 📞 문제 해결

문제 발생 시 확인사항:
1. `app_real.py` 터미널에서 오류 메시지 확인
2. 브라우저 개발자 도구 (F12) → Console 탭 확인
3. ROS2 토픽 확인: `ros2 topic list | grep dsr01`
4. 서비스 확인: `ros2 service list | grep dsr01`
