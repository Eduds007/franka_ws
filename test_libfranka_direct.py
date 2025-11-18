#!/usr/bin/env python3
"""
Simple test to attempt robot connection with minimal libfranka usage
This attempts to connect and read basic robot state without control
"""

import subprocess
import sys
import os

def test_libfranka_connection():
    """Test direct libfranka connection"""
    
    # Create a simple C++ test program
    cpp_code = '''
#include <iostream>
#include <franka/robot.h>
#include <franka/exception.h>

int main() {
    try {
        std::cout << "Attempting to connect to robot at 172.16.0.3..." << std::endl;
        franka::Robot robot("172.16.0.3");
        std::cout << "✓ Successfully connected to robot!" << std::endl;
        
        // Try to read robot state once
        auto state = robot.readOnce();
        std::cout << "✓ Successfully read robot state!" << std::endl;
        std::cout << "Robot mode: " << static_cast<int>(state.robot_mode) << std::endl;
        
        return 0;
    } catch (const franka::Exception& e) {
        std::cout << "✗ Franka exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cout << "✗ Exception: " << e.what() << std::endl;
        return 2;
    }
}
'''
    
    # Write the test program
    with open('/tmp/test_franka_connection.cpp', 'w') as f:
        f.write(cpp_code)
    
    print("🔍 Compiling libfranka connection test...")
    
    # Try to compile and run the test
    try:
        # Source ROS environment and compile
        compile_cmd = [
            'bash', '-c', 
            'source /opt/ros/humble/setup.bash && ' +
            'g++ -std=c++17 /tmp/test_franka_connection.cpp -o /tmp/test_franka_connection ' +
            '-I/opt/ros/humble/include -L/opt/ros/humble/lib -lfranka'
        ]
        
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"✗ Compilation failed: {result.stderr}")
            return False
            
        print("✓ Compilation successful")
        print("🔍 Testing robot connection...")
        
        # Run the test
        run_result = subprocess.run(['/tmp/test_franka_connection'], 
                                   capture_output=True, text=True, timeout=10)
        
        print("📊 Connection test output:")
        print(run_result.stdout)
        if run_result.stderr:
            print("Errors:")
            print(run_result.stderr)
            
        return run_result.returncode == 0
        
    except subprocess.TimeoutExpired:
        print("✗ Connection test timed out")
        return False
    except Exception as e:
        print(f"✗ Test execution failed: {e}")
        return False

def main():
    print("=" * 60)
    print("🔬 Direct libfranka Connection Test")
    print("=" * 60)
    
    success = test_libfranka_connection()
    
    print("\n" + "=" * 60)
    print("📊 Final Results")
    print("=" * 60)
    
    if success:
        print("🎉 Robot communication is working at the libfranka level!")
        print("   The ROS2 issues may be related to configuration or FCI mode.")
    else:
        print("❌ Direct libfranka connection failed.")
        print("   This confirms the version compatibility or FCI mode issues.")

if __name__ == '__main__':
    main()
