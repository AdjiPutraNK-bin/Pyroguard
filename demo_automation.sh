#!/bin/bash
# Demo: PyroGuard Automated Training
# This script demonstrates the automation capabilities

echo "🔥 PyroGuard DQN Training Automation Demo"
echo "=========================================="
echo ""

# Function to demonstrate each automation method
demo_bash() {
    echo "📋 Method 1: Bash Script Automation"
    echo "-----------------------------------"
    echo "Commands available:"
    echo "  ./auto_train.sh all           # Complete pipeline"
    echo "  ./auto_train.sh collect 5     # Collect for 5 min"
    echo "  ./auto_train.sh train 10      # Train for 10 min"
    echo "  ./auto_train.sh inference 3   # Test for 3 min"
    echo ""
}

demo_python() {
    echo "🐍 Method 2: Python Orchestrator"
    echo "-------------------------------"
    echo "Commands available:"
    echo "  python3 auto_train.py                    # Complete pipeline"
    echo "  python3 auto_train.py collect            # Collect only"
    echo "  python3 auto_train.py train_online       # Train only"
    echo "  python3 auto_train.py --duration 15      # 15 min phases"
    echo "  python3 auto_train.py --durations 5 10 3 # Custom durations"
    echo ""
}

demo_manual() {
    echo "🎮 Method 3: Manual ROS2 Commands"
    echo "---------------------------------"
    echo "Commands available:"
    echo "  ros2 launch pyroguard dqn_agent.launch.py mode:=collect"
    echo "  ros2 launch pyroguard dqn_agent.launch.py mode:=train_online"
    echo "  ros2 launch pyroguard dqn_agent.launch.py mode:=inference"
    echo ""
}

demo_monitoring() {
    echo "📊 Method 4: Training Monitoring"
    echo "-------------------------------"
    echo "Commands available:"
    echo "  python3 monitor_training.py    # Real-time monitoring"
    echo "  # Shows: steps/sec, rewards, episodes, fire detections"
    echo ""
}

# Run demonstrations
demo_bash
demo_python
demo_manual
demo_monitoring

echo "🚀 Quick Start Examples:"
echo "========================"
echo ""
echo "1. Run complete automated pipeline:"
echo "   ./auto_train.sh all"
echo ""
echo "2. Train with monitoring:"
echo "   python3 auto_train.py train_online --duration 10 &"
echo "   python3 monitor_training.py"
echo ""
echo "3. Quick test cycle:"
echo "   ./auto_train.sh collect 2"
echo "   ./auto_train.sh train 5"
echo "   ./auto_train.sh inference 2"
echo ""
echo "4. Background training:"
echo "   nohup ./auto_train.sh all > training.log 2>&1 &"
echo "   tail -f training.log"
echo ""

echo "📚 For detailed documentation, see: AUTOMATION_README.md"
echo "🎯 Happy automating! 🚀"
