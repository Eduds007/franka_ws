#!/usr/bin/env python3

import sys
import socket
import subprocess

def test_network_connectivity(robot_ip):
    """Test basic network connectivity to the robot"""
    print(f"🔍 Testing network connectivity to {robot_ip}...")
    
    # Test ping
    try:
        result = subprocess.run(['ping', '-c', '3', robot_ip], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ Robot at {robot_ip} is reachable via ping")
            return True
        else:
            print(f"✗ Robot at {robot_ip} is not reachable via ping")
            return False
    except subprocess.TimeoutExpired:
        print(f"✗ Ping to {robot_ip} timed out")
        return False
    except Exception as e:
        print(f"✗ Ping test failed: {e}")
        return False

def test_robot_ports(robot_ip):
    """Test if robot ports are accessible"""
    ports_to_test = [
        (1337, "FCI Control"),
        (8080, "Web Interface"),
        (1338, "Legacy Control")
    ]
    
    results = {}
    
    for port, description in ports_to_test:
        print(f"🔍 Testing {description} on port {port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        
        try:
            result = sock.connect_ex((robot_ip, port))
            if result == 0:
                print(f"✓ Port {port} ({description}) is open")
                results[port] = True
            else:
                print(f"✗ Port {port} ({description}) is closed/unreachable")
                results[port] = False
        except Exception as e:
            print(f"✗ Port {port} test failed: {e}")
            results[port] = False
        finally:
            sock.close()
    
    return results

def main():
    robot_ip = "172.16.0.3"
    
    print("=" * 50)
    print("🤖 Franka Robot Connection Test")
    print("=" * 50)
    
    # Test network connectivity
    network_ok = test_network_connectivity(robot_ip)
    
    if network_ok:
        # Test robot ports
        port_results = test_robot_ports(robot_ip)
        
        print("\n" + "=" * 50)
        print("📊 Test Results Summary")
        print("=" * 50)
        
        if port_results.get(1337, False):
            print("✓ FCI port accessible - Robot communication should work")
            print("⚠️  However, you may need to:")
            print("   1. Enable FCI mode in Franka Desk")
            print("   2. Check libfranka version compatibility")
        elif any(port_results.values()):
            print("⚠️  Robot is reachable but FCI port is not accessible")
            print("   - Check if FCI is enabled in Franka Desk")
        else:
            print("✗ Robot is reachable but no control ports are accessible")
            
        if port_results.get(8080, False):
            print(f"🌐 Robot web interface available at: http://{robot_ip}:8080")
            
    else:
        print("\n❌ Network connectivity failed!")
        print("   - Check if robot IP is correct")
        print("   - Check network configuration")
        print("   - Check if robot is powered on")

if __name__ == '__main__':
    main()
