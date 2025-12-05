#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
컨베이어 벨트 전용 모니터링 / 제어 웹 서버 (반응형 + 이모지 스타일 + 상태/DETECT 분리)
- Arduino + TB6600 + ROS2 브릿지 기반
"""

from std_msgs.msg import String, Int32, Bool

from flask import Flask, Response
from flask_socketio import SocketIO
import threading
import time
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ─────────────────────────────────────
# Flask / SocketIO
# ─────────────────────────────────────
app = Flask(__name__)
app.config['SECRET_KEY'] = 'conveyor_monitor_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ─────────────────────────────────────
# 전역 상태 저장
# ─────────────────────────────────────
conveyor_state = {
    "is_running": False,      # 실제 동작 여부 (Arduino → ROS)
    "state_label": "UNKNOWN", # RUNNING / STOPPED / ESTOP / UNKNOWN
    "last_cmd": "",           # 마지막으로 전송한 명령 (SPD1, SPD2, STOP, ESTOP, ...)
    "last_detect_id": 0,
    "last_detect_time": None,
    "total_detect": 0,
    "last_status": "",
}

logs = []      # 최근 100개의 로그 저장
ros_node = None


def get_kst():
    return datetime.now(timezone(timedelta(hours=9))).strftime("%H:%M:%S")


def add_log(level, message):
    """웹/콘솔 공용 로그 함수"""
    logs.insert(0, {
        "time": get_kst(),
        "level": level,
        "message": message
    })
    if len(logs) > 100:
        logs.pop()

    # 웹으로 최신 50개 전송
    socketio.emit("logs", logs[:50])


# ─────────────────────────────────────
# ROS2 노드
# ─────────────────────────────────────
class ConveyorMonitorNode(Node):

    def __init__(self):
        super().__init__("conveyor_monitor")
        self.callback_group = ReentrantCallbackGroup()

        # Publish (web → arduino)
        self.cmd_pub = self.create_publisher(String, "/conveyor/cmd", 10)

        # Subscribe (arduino → web)
        self.create_subscription(Int32, "/conveyor/detect",
                                 self.cb_detect, 10)
        self.create_subscription(Bool, "/conveyor/is_running",
                                 self.cb_running, 10)
        self.create_subscription(String, "/conveyor/status",
                                 self.cb_status, 10)
        self.create_subscription(String, "/conveyor/log",
                                 self.cb_log, 10)

        add_log("INFO", "컨베이어 모니터링 시작")

    def _update_state_label(self):
        """is_running + last_cmd를 기반으로 상위 상태(state_label) 계산"""
        global conveyor_state
        is_run = conveyor_state.get("is_running", False)
        last_cmd = (conveyor_state.get("last_cmd") or "").upper()

        if is_run:
            conveyor_state["state_label"] = "RUNNING"
        else:
            if last_cmd == "ESTOP":
                conveyor_state["state_label"] = "ESTOP"
            elif last_cmd in ("STOP", "SPD0"):
                conveyor_state["state_label"] = "STOPPED"
            else:
                # 외부에서 멈추거나 알 수 없는 경우
                conveyor_state["state_label"] = "STOPPED"

    def cb_detect(self, msg):
        global conveyor_state
        conveyor_state["last_detect_id"] = msg.data
        conveyor_state["total_detect"] += 1
        conveyor_state["last_detect_time"] = get_kst()

        socketio.emit("conveyor_state", conveyor_state)
        socketio.emit("conveyor_detect", conveyor_state)

        add_log("INFO", f"[DETECT] 감지 ID={msg.data}")

    def cb_running(self, msg):
        global conveyor_state
        conveyor_state["is_running"] = bool(msg.data)
        self._update_state_label()
        socketio.emit("conveyor_state", conveyor_state)

    def cb_status(self, msg):
        global conveyor_state
        conveyor_state["last_status"] = msg.data
        socketio.emit("conveyor_state", conveyor_state)
        socketio.emit("conveyor_status", {"raw": msg.data})

    def cb_log(self, msg):
        add_log("INFO", f"[CONVEYOR] {msg.data}")

    def send_command(self, cmd: str):
        """웹에서 들어온 명령을 아두이노로 전송"""
        global conveyor_state
        cmd = (cmd or "").strip().upper()
        if not cmd:
            return

        conveyor_state["last_cmd"] = cmd
        self._update_state_label()

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

        add_log("INFO", f"[SEND] {cmd}")


# ─────────────────────────────────────
# ROS2 스핀 스레드
# ─────────────────────────────────────
def ros_spin(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


# ─────────────────────────────────────
# 주기적 웹 송신
# ─────────────────────────────────────
def emit_state():
    while True:
        socketio.emit("conveyor_state", conveyor_state)
        time.sleep(0.15)


# ─────────────────────────────────────
# HTML 페이지
# ─────────────────────────────────────
@app.route("/")
def index():
    html = """
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>🚚 컨베이어 모니터링 대시보드 ✨</title>
<script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>

<style>
    body {
        font-family: "Pretendard", system-ui, sans-serif;
        background: #0f172a;
        color: #e5e7eb;
        margin: 0;
    }

    .container {
        width: 100%;
        max-width: 1400px;
        margin: 0 auto;
        padding: 24px 32px;
        box-sizing: border-box;
    }

    h1 { font-size: 28px; margin-bottom: 6px; font-weight: 700; }
    .subtitle { color:#94a3b8; margin-bottom: 18px; }

    .card {
        background: #0b1120;
        border-radius: 14px;
        border: 1px solid #1e293b;
        padding: 18px;
        box-sizing: border-box;
        box-shadow: 0 0 0 1px rgba(15,23,42,0.4), 0 12px 30px rgba(0,0,0,0.45);
    }

    .card h2 {
        font-size: 18px;
        font-weight: 600;
        margin-bottom: 14px;
        display: flex;
        align-items: center;
        gap: 6px;
    }

    /* 상단 버튼 바 */
    .control-bar {
        display: flex;
        flex-wrap: wrap;
        gap: 10px;
        justify-content: space-between;
        align-items: center;
    }

    .btn-group {
        display: flex;
        flex-wrap: wrap;
        gap: 10px;
    }

    button {
        padding: 10px 18px;
        background:#020617;
        color:#e5e7eb;
        border-radius:10px;
        border:1px solid #334155;
        cursor:pointer;
        transition:0.2s;
        display:flex;
        align-items:center;
        gap:6px;
        font-size: 14px;
        font-weight: 500;
    }
    button:hover { background:#1e293b; }
    .primary { background:#22c55e; border-color:#22c55e; color:#02140a; font-weight:700; }
    .danger { background:#ef4444; border-color:#ef4444; color:white; font-weight:700; }

    /* 중간 영역: 상태/DETECT vs 로그 */
    .layout-main {
        display: flex;
        gap: 18px;
        margin-top: 18px;
        align-items: flex-start;
    }
    .left-column {
        flex: 1 1 auto;
        display: flex;
        flex-direction: column;
        gap: 16px;
    }
    .right-column {
        flex: 0 0 auto;
        max-width: 360px;
        width: 100%;
        align-self: flex-end;
    }

    /* 상태 칩 */
    .status-chip {
        padding: 6px 14px;
        border-radius: 999px;
        font-weight: 700;
        font-size: 14px;
    }
    .running { background: rgba(34,197,94,0.16); color:#4ade80; }
    .stopped { background: rgba(148,163,184,0.16); color:#e5e7eb; }
    .estop { background: rgba(248,113,113,0.16); color:#fb7185; }
    .unknown { background: rgba(148,163,184,0.16); color:#cbd5e1; }

    .stat-row { display:flex; justify-content:space-between; margin-bottom:8px; font-size:14px; }
    .stat-label { color:#9ca3af; }
    .stat-value { font-weight:600; }

    .pill {
        padding: 4px 10px;
        border-radius: 999px;
        border: 1px solid #334155;
        background:#020617;
        font-size:13px;
        display:inline-flex;
        align-items:center;
        gap:6px;
        color:#9ca3af;
    }

    /* 로그 영역 */
    .log-area {
        background:#020617;
        height: 260px;
        padding:8px;
        border-radius:10px;
        border:1px solid #1e293b;
        overflow-y:auto;
        font-size: 12px;
    }
    .log-line { display:flex; gap:12px; margin-bottom:4px; }
    .log-time { width:60px; color:#64748b; }
    .LEVEL-INFO { color:#22c55e; }
    .LEVEL-WARN { color:#fbbf24; }
    .LEVEL-ERROR { color:#f87171; }

    @media (max-width: 900px) {
        .layout-main {
            flex-direction: column;
        }
        .right-column {
            max-width: 100%;
        }
    }
</style>
</head>

<body>
<div class="container">
    <h1>🚚 컨베이어 모니터링 대시보드 ✨</h1>
    <div class="subtitle">Arduino + TB6600 + ROS2 기반 실시간 컨베이어 제어/감지</div>

    <!-- 상단 버튼 바 -->
    <div class="card">
        <h2>🎛 제어 패널</h2>
        <div class="control-bar">
            <div class="btn-group">
                <!-- SPD0 제거, SPD1 / SPD2만 유지 -->
                <button onclick="sendCmd('SPD1')">🚶 SPD1 (저속)</button>
                <button onclick="sendCmd('SPD2')">🏃 SPD2 (중속)</button>
            </div>

            <div class="btn-group">
                <button class="primary" onclick="sendCmd('RESUME')">▶ RESUME</button>
                <button onclick="sendCmd('STOP')">⏹ STOP</button>
                <button onclick="sendCmd('STATUS')">📡 STATUS</button>
            </div>

            <div class="btn-group">
                <button onclick="sendCmd('BOOT')">🔄 BOOT</button>
                <button class="danger" onclick="sendCmd('ESTOP')">🧯 ESTOP</button>
                <button onclick="sendCmd('RESET')">♻ RESET</button>
            </div>
        </div>
    </div>

    <!-- 상태 / DETECT / 로그 레이아웃 -->
    <div class="layout-main">

        <!-- 왼쪽: 상태 + DETECT -->
        <div class="left-column">

            <!-- 상태 카드 -->
            <div class="card">
                <h2>📡 현재 상태</h2>
                <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:14px;">
                    <div id="status-chip" class="status-chip unknown">⚪ 상태: UNKNOWN</div>
                    <div class="pill">
                        ⚙ STATUS: <span id="last-status">-</span>
                    </div>
                </div>
                <div class="stat-row">
                    <span class="stat-label">실시간 동작 여부</span>
                    <span class="stat-value" id="running-flag">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">마지막 명령</span>
                    <span class="stat-value" id="last-cmd">-</span>
                </div>
            </div>

            <!-- DETECT 카드 -->
            <div class="card">
                <h2>🔍 DETECT 정보</h2>
                <div class="stat-row">
                    <span class="stat-label">마지막 DETECT ID</span>
                    <span class="stat-value" id="last-detect-id">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">감지 시간</span>
                    <span class="stat-value" id="last-detect-time">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">총 감지 횟수</span>
                    <span class="stat-value" id="total-detect">0</span>
                </div>
            </div>

        </div>

        <!-- 오른쪽: 이벤트 로그 -->
        <div class="right-column">
            <div class="card">
                <h2>📜 이벤트 로그</h2>
                <div class="log-area" id="log-area"></div>
            </div>
        </div>
    </div>
</div>

<script>
    const socket = io();

    function sendCmd(c) {
        socket.emit("conveyor_command", { cmd:c });
    }

    function updateStatusChip(stateLabel) {
        const el = document.getElementById("status-chip");
        el.classList.remove("running","stopped","estop","unknown");

        const label = (stateLabel || "UNKNOWN").toUpperCase();

        if (label === "RUNNING") {
            el.classList.add("running");
            el.textContent = "🟢 상태: RUNNING";
        } else if (label === "ESTOP") {
            el.classList.add("estop");
            el.textContent = "🧯 상태: ESTOP";
        } else if (label === "STOPPED") {
            el.classList.add("stopped");
            el.textContent = "⏹ 상태: STOPPED";
        } else {
            el.classList.add("unknown");
            el.textContent = "⚪ 상태: UNKNOWN";
        }
    }

    socket.on("conveyor_state", s => {
        if (!s) return;

        updateStatusChip(s.state_label);
        document.getElementById("running-flag").textContent =
            s.is_running ? "🟢 RUNNING" : "⏹ STOPPED";
        document.getElementById("last-cmd").textContent = s.last_cmd || "-";

        document.getElementById("last-status").textContent = s.last_status || "-";
        document.getElementById("last-detect-id").textContent = (s.last_detect_id ?? "-");
        document.getElementById("last-detect-time").textContent = s.last_detect_time || "-";
        document.getElementById("total-detect").textContent = (s.total_detect ?? 0);
    });

    socket.on("logs", list => {
        let area = document.getElementById("log-area");
        area.innerHTML = "";
        (list || []).forEach(e => {
            area.innerHTML += `
                <div class="log-line">
                    <span class="log-time">${e.time}</span>
                    <span class="LEVEL-${e.level}">${e.level}</span>
                    <span>${e.message}</span>
                </div>`;
        });
    });
</script>
</body>
</html>
"""
    return Response(html, mimetype="text/html")


# ─────────────────────────────────────
# SocketIO 이벤트
# ─────────────────────────────────────
@socketio.on("conveyor_command")
def socket_cmd(data):
    cmd = data.get("cmd", "").upper()
    if ros_node:
        ros_node.send_command(cmd)


# ─────────────────────────────────────
# main
# ─────────────────────────────────────
if __name__ == "__main__":
    print("🚀 Conveyor Dashboard Running at http://localhost:5001")

    rclpy.init()
    ros_node = ConveyorMonitorNode()

    threading.Thread(target=ros_spin, args=(ros_node,), daemon=True).start()
    threading.Thread(target=emit_state, daemon=True).start()

    socketio.run(app, host="0.0.0.0", port=5001, debug=False)
