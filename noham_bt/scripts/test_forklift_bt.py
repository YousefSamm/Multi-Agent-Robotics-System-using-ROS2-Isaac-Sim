#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET


def test_forklift_bt_xml():
    """Test if the forklift behavior tree XML can be loaded and parsed."""
    
    try:
        # Get package directory
        package_dir = get_package_share_directory('noham_bt')
        xml_path = os.path.join(package_dir, 'config',
                                'forklift_mission_tree.xml')
        
        print(f"Testing forklift behavior tree XML: {xml_path}")
        
        # Check if file exists
        if not os.path.exists(xml_path):
            print(f"❌ XML file not found: {xml_path}")
            return False
        
        # Parse XML
        tree = ET.parse(xml_path)
        root = tree.getroot()
        
        # Check BTCPP format
        if root.get('BTCPP_format') != '4':
            print("❌ BTCPP format should be 4")
            return False
        
        # Check main tree
        if root.get('main_tree_to_execute') != 'ForkliftMissionTree':
            print("❌ Main tree should be 'ForkliftMissionTree'")
            return False
        
        # Check behavior tree structure
        behavior_tree = root.find('BehaviorTree')
        if (behavior_tree is None or
                behavior_tree.get('ID') != 'ForkliftMissionTree'):
            print("❌ BehaviorTree with ID 'ForkliftMissionTree' not found")
            return False
        
        # Check sequence
        sequence = behavior_tree.find('Sequence')
        if sequence is None or sequence.get('name') != 'ForkliftMission':
            print("❌ Sequence 'ForkliftMission' not found")
            return False
        
        # Count actions
        actions = sequence.findall('Action')
        if len(actions) != 6:
            print(f"❌ Expected 6 actions, found {len(actions)}")
            return False
        
        print("✅ Forklift behavior tree XML is valid!")
        print(f"   - Found {len(actions)} actions in sequence")
        
        # List actions
        for i, action in enumerate(actions, 1):
            name = action.get('name', 'unnamed')
            action_id = action.get('ID', 'unknown')
            print(f"   {i}. {name} ({action_id})")
        
        return True
        
    except Exception as e:
        print(f"❌ Error testing XML: {e}")
        return False


if __name__ == '__main__':
    success = test_forklift_bt_xml()
    sys.exit(0 if success else 1)
