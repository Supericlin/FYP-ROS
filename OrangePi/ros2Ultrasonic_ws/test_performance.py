#!/usr/bin/env python3
"""
Performance test script for OrangePi ultrasonic sensor node
"""

import subprocess
import time
import psutil
import os

def get_cpu_usage(pid):
    """Get CPU usage percentage for a process"""
    try:
        process = psutil.Process(pid)
        return process.cpu_percent(interval=1.0)
    except:
        return 0.0

def get_memory_usage(pid):
    """Get memory usage in MB for a process"""
    try:
        process = psutil.Process(pid)
        return process.memory_info().rss / 1024 / 1024  # Convert to MB
    except:
        return 0.0

def test_performance():
    """Test the performance of the OrangePi ultrasonic sensor node"""
    print("🚀 Testing OrangePi Ultrasonic Sensor Performance")
    print("=" * 60)
    
    # Build the package first
    print("📦 Building package...")
    try:
        subprocess.run(["colcon", "build", "--packages-select", "ultrasonic_sensors"], 
                      check=True, capture_output=True)
        print("✅ Build successful")
    except subprocess.CalledProcessError as e:
        print(f"❌ Build failed: {e}")
        return
    
    # Source the workspace
    os.system("source install/setup.bash")
    
    # Launch the node in background
    print("🎯 Launching OrangePi sensor node...")
    try:
        # Launch with optimized settings
        process = subprocess.Popen([
            "ros2", "launch", "ultrasonic_sensors", "orangepi_ultrasonic_launch.py",
            "log_interval:=30.0",
            "debug_mode:=false",
            "enable_detailed_logging:=false"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print(f"✅ Node launched with PID: {process.pid}")
        
        # Monitor performance for 60 seconds
        print("\n📊 Monitoring performance for 60 seconds...")
        print("Time(s) | CPU(%) | Memory(MB) | Status")
        print("-" * 45)
        
        start_time = time.time()
        cpu_readings = []
        memory_readings = []
        
        for i in range(60):  # Monitor for 60 seconds
            cpu = get_cpu_usage(process.pid)
            memory = get_memory_usage(process.pid)
            
            cpu_readings.append(cpu)
            memory_readings.append(memory)
            
            elapsed = time.time() - start_time
            print(f"{elapsed:6.1f} | {cpu:6.1f} | {memory:9.1f} | Running")
            
            time.sleep(1)
            
            # Check if process is still running
            if process.poll() is not None:
                print("❌ Process terminated unexpectedly")
                break
        
        # Calculate statistics
        avg_cpu = sum(cpu_readings) / len(cpu_readings) if cpu_readings else 0
        max_cpu = max(cpu_readings) if cpu_readings else 0
        avg_memory = sum(memory_readings) / len(memory_readings) if memory_readings else 0
        max_memory = max(memory_readings) if memory_readings else 0
        
        print("\n📈 PERFORMANCE SUMMARY")
        print("=" * 30)
        print(f"Average CPU Usage: {avg_cpu:.2f}%")
        print(f"Maximum CPU Usage: {max_cpu:.2f}%")
        print(f"Average Memory: {avg_memory:.1f} MB")
        print(f"Maximum Memory: {max_memory:.1f} MB")
        
        # Kill the process
        process.terminate()
        process.wait(timeout=5)
        print("\n✅ Performance test completed")
        
    except Exception as e:
        print(f"❌ Error during performance test: {e}")
        if 'process' in locals():
            process.terminate()

if __name__ == "__main__":
    test_performance() 