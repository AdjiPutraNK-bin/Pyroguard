#!/usr/bin/env python3
"""
Automated DQN Training Orchestrator for PyroGuard
Provides advanced automation with monitoring and adaptive training
"""

import subprocess
import time
import signal
import sys
import os
from datetime import datetime, timedelta
import psutil
import json

class DQNAutomator:
    def __init__(self):
        self.process = None
        self.start_time = None
        self.phase_logs = []

    def log(self, message, level="INFO"):
        """Log messages with timestamps"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        colored_message = self._colorize_message(message, level)
        print(f"[{timestamp}] {colored_message}")

        self.phase_logs.append({
            'timestamp': timestamp,
            'level': level,
            'message': message
        })

    def _colorize_message(self, message, level):
        """Add colors to log messages"""
        colors = {
            'INFO': '\033[0;34m',    # Blue
            'SUCCESS': '\033[0;32m', # Green
            'WARNING': '\033[1;33m', # Yellow
            'ERROR': '\033[0;31m',   # Red
            'RESET': '\033[0m'       # Reset
        }

        color = colors.get(level, colors['RESET'])
        return f"{color}{message}{colors['RESET']}"

    def run_command(self, cmd, timeout=None):
        """Run a command with optional timeout"""
        self.log(f"Executing: {cmd}")

        try:
            self.process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            if timeout:
                self.process.wait(timeout=timeout)
            else:
                self.process.wait()

            return self.process.returncode == 0

        except subprocess.TimeoutExpired:
            self.log("Command timed out", "WARNING")
            return False
        except Exception as e:
            self.log(f"Command failed: {e}", "ERROR")
            return False

    def launch_training(self, mode, duration_minutes=None):
        """Launch training with specified mode"""
        cmd = f"ros2 launch pyroguard dqn_agent.launch.py mode:={mode}"

        self.start_time = datetime.now()
        self.log(f"🚀 Starting {mode.upper()} phase")

        if duration_minutes:
            timeout_seconds = duration_minutes * 60
            success = self.run_command(cmd, timeout=timeout_seconds)
        else:
            success = self.run_command(cmd)

        end_time = datetime.now()
        duration = end_time - self.start_time

        if success:
            self.log(f"✅ {mode.upper()} completed successfully in {duration}", "SUCCESS")
        else:
            self.log(f"❌ {mode.upper()} failed or was interrupted", "ERROR")

        return success

    def check_model_exists(self):
        """Check if trained model exists"""
        model_path = "dqn_model.pth"
        exists = os.path.exists(model_path)

        if exists:
            file_size = os.path.getsize(model_path)
            self.log(f"📁 Found model: {model_path} ({file_size} bytes)", "SUCCESS")
        else:
            self.log("📁 No trained model found", "WARNING")

        return exists

    def get_system_stats(self):
        """Get system resource usage"""
        try:
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            return {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_used_gb': memory.used / (1024**3)
            }
        except:
            return None

    def run_pipeline(self, phases=None, durations=None):
        """Run the complete training pipeline"""
        if phases is None:
            phases = ['collect', 'train_online', 'inference']

        if durations is None:
            durations = [5, 10, 5]  # Default durations in minutes

        self.log("🔥 Starting PyroGuard DQN Training Pipeline")
        self.log("=" * 50)

        # Show system stats
        stats = self.get_system_stats()
        if stats:
            self.log(f"💻 System: CPU {stats['cpu_percent']}%, RAM {stats['memory_percent']:.1f}%")

        results = {}

        for i, phase in enumerate(phases):
            duration = durations[i] if i < len(durations) else None

            if phase == 'inference' and not self.check_model_exists():
                self.log("⏭️  Skipping inference - no trained model available", "WARNING")
                continue

            success = self.launch_training(phase, duration)
            results[phase] = success

            # Brief pause between phases
            if i < len(phases) - 1:
                self.log("⏰ Cooling down for 10 seconds...")
                time.sleep(10)

        # Summary
        self.log("\n📊 Pipeline Summary:")
        for phase, success in results.items():
            status = "✅ SUCCESS" if success else "❌ FAILED"
            self.log(f"  {phase.upper()}: {status}")

        # Save logs
        self.save_logs()

        return results

    def save_logs(self):
        """Save training logs to file"""
        log_file = f"training_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

        log_data = {
            'timestamp': datetime.now().isoformat(),
            'phases': self.phase_logs,
            'system_info': self.get_system_stats()
        }

        with open(log_file, 'w') as f:
            json.dump(log_data, f, indent=2)

        self.log(f"📝 Logs saved to: {log_file}", "SUCCESS")

def main():
    import argparse

    parser = argparse.ArgumentParser(description='PyroGuard DQN Training Automator')
    parser.add_argument('phases', nargs='*', default=['collect', 'train_online', 'inference'],
                       help='Phases to run (collect, train_online, inference)')
    parser.add_argument('--duration', '-d', type=int, default=5,
                       help='Duration per phase in minutes')
    parser.add_argument('--durations', nargs='*', type=int,
                       help='Specific durations for each phase')

    args = parser.parse_args()

    # Handle keyboard interrupt gracefully
    def signal_handler(sig, frame):
        print('\n🛑 Interrupted by user')
        if automator.process:
            os.killpg(os.getpgid(automator.process.pid), signal.SIGTERM)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    automator = DQNAutomator()

    # Set durations
    if args.durations:
        durations = args.durations
    else:
        durations = [args.duration] * len(args.phases)

    # Run the pipeline
    results = automator.run_pipeline(args.phases, durations)

    # Exit with appropriate code
    if all(results.values()):
        print("\n🎉 All phases completed successfully!")
        sys.exit(0)
    else:
        print("\n⚠️  Some phases failed")
        sys.exit(1)

if __name__ == '__main__':
    main()
