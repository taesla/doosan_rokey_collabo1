#!/bin/bash
# 물류 로봇 시스템 시작 스크립트
# 기존 프로세스 정리 + 빌드 확인 + 실행

set -e

WORKSPACE="/home/taesla/cobot1_ws"
cd "$WORKSPACE"

echo "=========================================="
echo "🚀 물류 로봇 시스템 시작"
echo "=========================================="

# 1. 기존 프로세스 완전 종료
echo ""
echo "🛑 Step 1: 기존 프로세스 종료..."
pkill -9 -f "web_server_node" 2>/dev/null || true
pkill -9 -f "sort_node" 2>/dev/null || true
pkill -9 -f "dlar_sort_node" 2>/dev/null || true
pkill -9 -f "dsr01" 2>/dev/null || true
pkill -9 -f "ros2.*launch" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 2
echo "   ✅ 프로세스 종료 완료"

# 2. 포트 5000 확인
echo ""
echo "🔍 Step 2: 포트 5000 확인..."
if lsof -i :5000 > /dev/null 2>&1; then
    echo "   ⚠️ 포트 5000 사용 중 - 강제 종료"
    fuser -k 5000/tcp 2>/dev/null || true
    sleep 1
fi
echo "   ✅ 포트 5000 사용 가능"

# 3. 환경 설정
echo ""
echo "📦 Step 3: ROS2 환경 설정..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
echo "   ✅ 환경 설정 완료"

# 4. 핵심 파일 검증
echo ""
echo "🔎 Step 4: 핵심 파일 검증..."

# state_manager.py 검증
STATE_MANAGER="$WORKSPACE/src/dsr_integrated/dsr_integrated/monitoring/state_manager.py"
if [ ! -s "$STATE_MANAGER" ]; then
    echo "   ❌ 오류: state_manager.py가 비어있습니다!"
    echo "   파일을 복구해야 합니다."
    exit 1
fi
echo "   ✅ state_manager.py 정상"

# index_full.html 검증
INDEX_FULL="$WORKSPACE/src/logistics_monitor/templates/index_full.html"
if [ ! -s "$INDEX_FULL" ]; then
    echo "   ❌ 오류: index_full.html이 비어있습니다!"
    exit 1
fi
echo "   ✅ index_full.html 정상"

# 5. 시스템 실행
echo ""
echo "🚀 Step 5: 시스템 실행..."
echo "=========================================="
echo ""

ros2 launch dsr_integrated full_system.launch.py
