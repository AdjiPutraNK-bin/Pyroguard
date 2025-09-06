# 🔥 PyroGuard DQN Training Automation

This guide shows you how to automatically run the DQN training pipeline without manual intervention.

## 🚀 Quick Start

### Option 1: Simple Bash Script
```bash
# Run complete pipeline (collect → train → inference)
./auto_train.sh all

# Run individual phases
./auto_train.sh collect 10    # Collect for 10 minutes
./auto_train.sh train 15      # Train for 15 minutes
./auto_train.sh inference 5   # Test for 5 minutes
```

### Option 2: Advanced Python Orchestrator
```bash
# Run complete pipeline with monitoring
python3 auto_train.py

# Run specific phases with custom durations
python3 auto_train.py collect train_online --durations 10 20

# Run only training phase
python3 auto_train.py train_online --duration 30
```

### Option 3: Manual Launch Commands
```bash
# Phase 1: Collect experiences
ros2 launch pyroguard dqn_agent.launch.py mode:=collect

# Phase 2: Train online
ros2 launch pyroguard dqn_agent.launch.py mode:=train_online

# Phase 3: Deploy model
ros2 launch pyroguard dqn_agent.launch.py mode:=inference
```

## 📊 Monitoring Training Progress

### Real-time Monitoring
```bash
# Monitor training in real-time
python3 monitor_training.py
```

This will show:
- ✅ Steps per second
- ✅ Episodes completed
- ✅ Average rewards
- ✅ Fire detection count
- ✅ Training summary on exit

## 🎯 Automation Scripts Overview

### `auto_train.sh` - Simple Bash Automation
- ✅ **Easy to use** - Single command execution
- ✅ **Colorized output** - Clear progress indication
- ✅ **Flexible timing** - Custom duration per phase
- ✅ **Error handling** - Automatic cleanup on failure

### `auto_train.py` - Advanced Python Orchestrator
- ✅ **Detailed logging** - JSON logs with timestamps
- ✅ **System monitoring** - CPU/RAM usage tracking
- ✅ **Adaptive training** - Smart phase skipping
- ✅ **Progress tracking** - Real-time statistics
- ✅ **Graceful shutdown** - Proper cleanup on interrupt

### `monitor_training.py` - Training Monitor
- ✅ **Real-time feedback** - Live training statistics
- ✅ **Performance metrics** - Steps/sec, episode rewards
- ✅ **Fire detection tracking** - Training effectiveness
- ✅ **Summary reports** - Complete training overview

## 🔧 Advanced Usage

### Custom Training Pipeline
```bash
# Run only collect and train phases
python3 auto_train.py collect train_online --durations 5 15

# Skip inference, focus on data collection
python3 auto_train.py collect --duration 20

# Long training session with monitoring
python3 auto_train.py train_online --duration 60 &
python3 monitor_training.py
```

### Background Training
```bash
# Run training in background
nohup ./auto_train.sh all > training.log 2>&1 &

# Monitor progress
tail -f training.log

# Check if still running
ps aux | grep auto_train
```

### Integration with Existing Scripts
```bash
# Use with your existing launch script
./run_all_nodes_normal.sh &
sleep 10  # Wait for nodes to start
./auto_train.sh train 30
```

## 📈 Training Phases Explained

### Phase 1: Collect (`mode:=collect`)
- **Purpose**: Gather training experiences
- **Duration**: 5-10 minutes recommended
- **Output**: `replay_buffer.pkl` (if interrupted)
- **Behavior**: Robot explores, collects (state, action, reward) tuples

### Phase 2: Train (`mode:=train_online`)
- **Purpose**: Train DQN model in real-time
- **Duration**: 10-30 minutes recommended
- **Output**: `dqn_model.pth` (saved every 1000 steps)
- **Behavior**: Learns from experiences, improves policy

### Phase 3: Inference (`mode:=inference`)
- **Purpose**: Test trained model
- **Duration**: 5-10 minutes recommended
- **Input**: `dqn_model.pth`
- **Behavior**: Uses learned policy, no exploration

## 🎮 Example Workflows

### Workflow 1: Fresh Training
```bash
# Start fresh - no existing model
./auto_train.sh all
```

### Workflow 2: Continue Training
```bash
# Continue training existing model
./auto_train.sh train 20
```

### Workflow 3: Data Collection Only
```bash
# Collect more training data
./auto_train.sh collect 15
```

### Workflow 4: Model Testing
```bash
# Test trained model performance
./auto_train.sh inference 10
```

## 📊 Monitoring & Logs

### Log Files Generated
- `training_log_YYYYMMDD_HHMMSS.json` - Detailed training logs
- `dqn_model.pth` - Trained model weights
- `replay_buffer.pkl` - Collected experiences (if using collect mode)

### Real-time Metrics
- **Steps/second**: Training speed indicator
- **Episode rewards**: Learning progress
- **Fire detections**: Task completion rate
- **CPU/RAM usage**: System resource monitoring

## 🚨 Troubleshooting

### Common Issues

**"No model found"**
```bash
# Solution: Start with collect phase or check model path
./auto_train.sh collect 5
```

**Training not starting**
```bash
# Check: Wait for replay buffer to fill (64+ experiences)
# Monitor: python3 monitor_training.py
```

**ROS2 nodes not responding**
```bash
# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

**Permission denied**
```bash
# Make scripts executable
chmod +x auto_train.sh auto_train.py monitor_training.py
```

## 🎯 Best Practices

1. **Start Small**: Begin with 5-minute phases to test
2. **Monitor Progress**: Use `monitor_training.py` to track learning
3. **Save Frequently**: Models auto-save every 1000 steps
4. **Check Resources**: Monitor CPU/RAM usage
5. **Clean Shutdown**: Use Ctrl+C for graceful termination

## 📚 Next Steps

- Experiment with different durations
- Monitor learning curves with `monitor_training.py`
- Adjust hyperparameters in launch files
- Integrate with your simulation environment
- Set up automated testing pipelines

Happy training! 🚀🔥
