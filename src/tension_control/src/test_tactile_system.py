#!/usr/bin/env python3
"""
Test script for Tactile Object Detection System
Tests the integration and functionality of the tactile feedback system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Int32
from sensor_msgs.msg import Image
import time
import threading
from enum import Enum


class TestResult(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    WARN = "WARN"


class TactileSystemTester(Node):
    """Test node for tactile detection system"""
    
    def __init__(self):
        super().__init__('tactile_system_tester')
        
        # Test results storage
        self.test_results = {}
        self.test_data = {
            'object_detected': None,
            'object_confidence': None,
            'grip_force': None,
            'detection_status': None,
            'object_location': None,
            'control_state': None,
            'camera1_received': False,
            'camera2_received': False,
            'messages_count': 0
        }
        
        # Subscribers for testing
        self.object_detected_sub = self.create_subscription(
            Bool, '/tension_control/object_detected', self.object_detected_callback, 10)
        self.object_confidence_sub = self.create_subscription(
            Float32, '/tension_control/object_confidence', self.object_confidence_callback, 10)
        self.grip_force_sub = self.create_subscription(
            Float32, '/tension_control/recommended_grip_force', self.grip_force_callback, 10)
        self.detection_status_sub = self.create_subscription(
            String, '/tension_control/detection_status', self.detection_status_callback, 10)
        self.object_location_sub = self.create_subscription(
            Int32, '/tension_control/object_location', self.object_location_callback, 10)
        self.control_state_sub = self.create_subscription(
            String, '/tension_control/control_state', self.control_state_callback, 10)
        
        # Camera subscribers
        self.cam1_sub = self.create_subscription(
            Image, '/gelsight_cam1/image_raw', self.camera1_callback, 10)
        self.cam2_sub = self.create_subscription(
            Image, '/gelsight_cam2/image_raw', self.camera2_callback, 10)
        
        self.get_logger().info("🧪 Tactile System Tester initialized")
    
    def object_detected_callback(self, msg):
        self.test_data['object_detected'] = msg.data
        self.test_data['messages_count'] += 1
    
    def object_confidence_callback(self, msg):
        self.test_data['object_confidence'] = msg.data
    
    def grip_force_callback(self, msg):
        self.test_data['grip_force'] = msg.data
    
    def detection_status_callback(self, msg):
        self.test_data['detection_status'] = msg.data
    
    def object_location_callback(self, msg):
        self.test_data['object_location'] = msg.data
    
    def control_state_callback(self, msg):
        self.test_data['control_state'] = msg.data
    
    def camera1_callback(self, msg):
        self.test_data['camera1_received'] = True
    
    def camera2_callback(self, msg):
        self.test_data['camera2_received'] = True
    
    def run_test(self, test_name, test_function, timeout=5.0):
        """Run a single test with timeout"""
        self.get_logger().info(f"🔍 Running test: {test_name}")
        
        start_time = time.time()
        try:
            result = test_function()
            elapsed = time.time() - start_time
            
            self.test_results[test_name] = {
                'result': result,
                'elapsed': elapsed,
                'error': None
            }
            
            status_symbol = {'PASS': '✅', 'FAIL': '❌', 'SKIP': '⏭️', 'WARN': '⚠️'}[result.value]
            self.get_logger().info(f"{status_symbol} {test_name}: {result.value} ({elapsed:.2f}s)")
            
        except Exception as e:
            elapsed = time.time() - start_time
            self.test_results[test_name] = {
                'result': TestResult.FAIL,
                'elapsed': elapsed,
                'error': str(e)
            }
            self.get_logger().error(f"❌ {test_name}: FAIL - {e}")
    
    def test_topic_availability(self):
        """Test if required topics are available"""
        try:
            topic_list = self.get_topic_names_and_types()
            required_topics = [
                '/tension_control/object_detected',
                '/tension_control/object_confidence',
                '/tension_control/detection_status'
            ]
            
            missing_topics = []
            for topic in required_topics:
                if not any(topic in t[0] for t in topic_list):
                    missing_topics.append(topic)
            
            if missing_topics:
                self.get_logger().warn(f"Missing topics: {missing_topics}")
                return TestResult.WARN
            
            return TestResult.PASS
            
        except Exception:
            return TestResult.FAIL
    
    def test_message_reception(self):
        """Test if messages are being received"""
        initial_count = self.test_data['messages_count']
        
        # Wait for messages
        for _ in range(50):  # 5 seconds at 10Hz
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.test_data['messages_count'] > initial_count:
                return TestResult.PASS
        
        return TestResult.FAIL
    
    def test_detection_data_validity(self):
        """Test if detection data is valid"""
        # Wait for some data
        for _ in range(30):
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Check confidence range
        if self.test_data['object_confidence'] is not None:
            conf = self.test_data['object_confidence']
            if not (0.0 <= conf <= 1.0):
                self.get_logger().error(f"Invalid confidence value: {conf}")
                return TestResult.FAIL
        
        # Check object location
        if self.test_data['object_location'] is not None:
            loc = self.test_data['object_location']
            if loc not in [0, 1, 2, 3]:
                self.get_logger().error(f"Invalid object location: {loc}")
                return TestResult.FAIL
        
        return TestResult.PASS
    
    def test_camera_feeds(self):
        """Test if camera feeds are available"""
        # Wait for camera data
        for _ in range(50):  # 5 seconds
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.test_data['camera1_received'] and self.test_data['camera2_received']:
            return TestResult.PASS
        elif self.test_data['camera1_received'] or self.test_data['camera2_received']:
            return TestResult.WARN
        else:
            return TestResult.FAIL
    
    def test_grip_force_recommendation(self):
        """Test grip force recommendation functionality"""
        # Wait for grip force data
        for _ in range(30):
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.test_data['grip_force'] is not None:
            force = self.test_data['grip_force']
            if 0.0 <= force <= 100.0:  # Reasonable range
                return TestResult.PASS
            else:
                return TestResult.WARN
        
        return TestResult.SKIP  # No grip force data (might be normal)
    
    def test_state_machine(self):
        """Test state machine functionality"""
        # Wait for control state data
        for _ in range(30):
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.test_data['control_state'] is not None:
            valid_states = ['idle', 'learning_background', 'monitoring', 
                          'object_detected', 'adjusting_grip', 'maintaining_tension', 'error']
            
            if self.test_data['control_state'] in valid_states:
                return TestResult.PASS
            else:
                return TestResult.WARN
        
        return TestResult.SKIP
    
    def run_all_tests(self):
        """Run all tests"""
        self.get_logger().info("🚀 Starting comprehensive tactile system tests...")
        
        tests = [
            ("Topic Availability", self.test_topic_availability),
            ("Message Reception", self.test_message_reception),
            ("Detection Data Validity", self.test_detection_data_validity),
            ("Camera Feeds", self.test_camera_feeds),
            ("Grip Force Recommendation", self.test_grip_force_recommendation),
            ("State Machine", self.test_state_machine),
        ]
        
        for test_name, test_func in tests:
            self.run_test(test_name, test_func)
        
        # Print summary
        self.print_test_summary()
    
    def print_test_summary(self):
        """Print test results summary"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 TEST SUMMARY")
        self.get_logger().info("=" * 60)
        
        total_tests = len(self.test_results)
        passed = sum(1 for r in self.test_results.values() if r['result'] == TestResult.PASS)
        failed = sum(1 for r in self.test_results.values() if r['result'] == TestResult.FAIL)
        warnings = sum(1 for r in self.test_results.values() if r['result'] == TestResult.WARN)
        skipped = sum(1 for r in self.test_results.values() if r['result'] == TestResult.SKIP)
        
        for test_name, result in self.test_results.items():
            status_symbol = {'PASS': '✅', 'FAIL': '❌', 'SKIP': '⏭️', 'WARN': '⚠️'}[result['result'].value]
            self.get_logger().info(f"{status_symbol} {test_name}: {result['result'].value} ({result['elapsed']:.2f}s)")
            if result['error']:
                self.get_logger().info(f"   Error: {result['error']}")
        
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"📈 Results: {passed} passed, {failed} failed, {warnings} warnings, {skipped} skipped")
        self.get_logger().info(f"📊 Success rate: {passed}/{total_tests} ({100*passed/total_tests:.1f}%)")
        
        if failed == 0:
            self.get_logger().info("🎉 All critical tests passed!")
        else:
            self.get_logger().warn(f"⚠️  {failed} test(s) failed - check system configuration")


def main():
    """Main test function"""
    rclpy.init()
    
    try:
        tester = TactileSystemTester()
        
        # Wait a moment for connections
        time.sleep(2.0)
        
        # Run tests
        tester.run_all_tests()
        
        # Keep running for a bit to collect more data
        tester.get_logger().info("🔄 Collecting additional data...")
        
        for i in range(10):
            rclpy.spin_once(tester, timeout_sec=0.5)
        
        tester.get_logger().info("🏁 Testing complete!")
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
