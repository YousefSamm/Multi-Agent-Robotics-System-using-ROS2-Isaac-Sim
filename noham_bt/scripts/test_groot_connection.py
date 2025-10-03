#!/usr/bin/env python3

import socket
import subprocess


def test_groot_port():
    """Test if port 1667 is available for Groot connection."""
    
    host = 'localhost'
    port = 1667
    
    print(f"Testing Groot connection on {host}:{port}")
    
    try:
        # Create a socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        
        # Try to connect
        result = sock.connect_ex((host, port))
        
        if result == 0:
            print("✅ Port 1667 is open and accessible")
            print("   Groot should be able to connect to this port")
            return True
        else:
            print("❌ Port 1667 is not accessible")
            print("   Make sure your behavior tree is running")
            return False
            
    except socket.error as e:
        print(f"❌ Socket error: {e}")
        return False
    finally:
        sock.close()


def test_behavior_tree_running():
    """Check if any behavior tree nodes are running."""
    
    try:
        # Check for running behavior tree nodes
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=5
        )
        
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            bt_nodes = [n for n in nodes if 'behavior_tree' in n]
            
            if bt_nodes:
                print("✅ Behavior tree nodes found:")
                for node in bt_nodes:
                    print(f"   - {node}")
                return True
            else:
                print("❌ No behavior tree nodes running")
                print("   Start a behavior tree first:")
                print("   ros2 launch noham_bt forklift_mission.launch.py")
                return False
        else:
            print("❌ Failed to get node list")
            return False
            
    except Exception as e:
        print(f"❌ Error checking nodes: {e}")
        return False


if __name__ == '__main__':
    print("🔍 Groot Connection Test")
    print("=" * 40)
    
    # Test if behavior tree is running
    bt_running = test_behavior_tree_running()
    
    if bt_running:
        print("\n" + "=" * 40)
        # Test port accessibility
        port_accessible = test_groot_port()
        
        if port_accessible:
            print("\n🎉 Ready for Groot connection!")
            print("1. Open Groot2")
            print("2. Click Connect button")
            print("3. Enter: localhost:1667")
            print("4. Start monitoring your behavior tree!")
        else:
            print("\n⚠️  Port test failed")
            print("   Check firewall settings or try a different port")
    else:
        print("\n⚠️  Start a behavior tree first")
        print("   Then run this test again")
    
    print("\n" + "=" * 40)
