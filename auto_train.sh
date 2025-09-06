#!/bin/bash
# Automated DQN Training Pipeline for PyroGuard
# Usage: bash auto_train.sh [phase] [duration_minutes]

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
PHASE=${1:-"all"}
DURATION=${2:-5}  # Default 5 minutes per phase

echo -e "${BLUE}🔥 PyroGuard Automated DQN Training${NC}"
echo -e "${BLUE}=====================================${NC}"

# Function to run a training phase
run_phase() {
    local phase_name=$1
    local mode=$2
    local duration=$3

    echo -e "${YELLOW}🚀 Starting Phase: ${phase_name}${NC}"
    echo -e "${YELLOW}Mode: ${mode}${NC}"
    echo -e "${YELLOW}Duration: ${duration} minutes${NC}"

    # Launch the training
    ros2 launch pyroguard dqn_agent.launch.py mode:=${mode} &
    LAUNCH_PID=$!

    # Wait for specified duration
    echo -e "${BLUE}⏰ Training for ${duration} minutes...${NC}"
    sleep $((duration * 60))

    # Stop the training
    echo -e "${RED}🛑 Stopping ${phase_name}...${NC}"
    kill $LAUNCH_PID 2>/dev/null || true

    # Wait a bit for cleanup
    sleep 5

    echo -e "${GREEN}✅ ${phase_name} completed${NC}"
    echo ""
}

# Function to check if model exists
check_model() {
    if [ -f "dqn_model.pth" ]; then
        echo -e "${GREEN}📁 Found existing model: dqn_model.pth${NC}"
        return 0
    else
        echo -e "${YELLOW}📁 No existing model found, starting fresh${NC}"
        return 1
    fi
}

# Main execution
case $PHASE in
    "collect")
        echo -e "${BLUE}🎯 Running COLLECT phase only${NC}"
        run_phase "COLLECT" "collect" $DURATION
        ;;

    "train")
        echo -e "${BLUE}🎯 Running TRAIN phase only${NC}"
        check_model
        run_phase "TRAIN" "train_online" $DURATION
        ;;

    "inference")
        echo -e "${BLUE}🎯 Running INFERENCE phase only${NC}"
        if check_model; then
            run_phase "INFERENCE" "inference" $DURATION
        else
            echo -e "${RED}❌ No trained model found! Run 'train' phase first.${NC}"
            exit 1
        fi
        ;;

    "all")
        echo -e "${BLUE}🎯 Running COMPLETE pipeline${NC}"

        # Phase 1: Collect
        run_phase "COLLECT" "collect" $DURATION

        # Phase 2: Train
        run_phase "TRAIN" "train_online" $DURATION

        # Phase 3: Inference (if model exists)
        if [ -f "dqn_model.pth" ]; then
            run_phase "INFERENCE" "inference" $DURATION
        fi

        echo -e "${GREEN}🎉 Complete training pipeline finished!${NC}"
        ;;

    *)
        echo -e "${RED}❌ Invalid phase: ${PHASE}${NC}"
        echo -e "${YELLOW}Usage: $0 [collect|train|inference|all] [duration_minutes]${NC}"
        echo -e "${YELLOW}Examples:${NC}"
        echo -e "${YELLOW}  $0 all           # Run complete pipeline${NC}"
        echo -e "${YELLOW}  $0 collect 10    # Collect for 10 minutes${NC}"
        echo -e "${YELLOW}  $0 train 15      # Train for 15 minutes${NC}"
        echo -e "${YELLOW}  $0 inference 5   # Test for 5 minutes${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}🏁 Automation complete!${NC}"
