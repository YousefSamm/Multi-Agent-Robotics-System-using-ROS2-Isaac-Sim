# Forklift In-Place Rotation Fix

## Problem Description
The forklift was attempting to rotate in place to reach goals, which is not suitable for Ackermann steering vehicles like forklifts. This behavior was caused by several configuration issues in the DWB (Dynamic Window Based) controller.

## Root Causes Identified
1. **Duplicate Controller Configuration**: Two different controller configurations were defined in separate files
2. **High Angular Velocity Limits**: Angular velocity was set too high (0.5 rad/s) allowing spinning
3. **Missing Rotation Prevention**: No specific parameters to prevent in-place rotation
4. **Inadequate Critic Weights**: DWB critics were not properly configured to prioritize forward movement
5. **Missing Behavior Tree Reference**: BT navigator wasn't referencing the behavior tree XML file

## Changes Made

### 1. forklift_controller.yaml - DWB Controller Optimization
- **Reduced Angular Velocity**: `max_vel_theta` from 0.5 to 0.3 rad/s
- **Added RotateToGoal Critic**: With very low weight (0.1) to minimize rotation preference
- **Increased Path Following Weights**: `PathDist.scale` and `PathAlign.scale` to 20.0
- **Added Rotation Penalties**: `angular_vel_cost: 2.0` to penalize high angular velocities
- **Improved Sampling**: Increased `vx_samples` to 15 and `vtheta_samples` to 7
- **Extended Simulation Time**: `sim_time` from 1.0 to 1.5 seconds for better trajectory evaluation
- **Added Forward Bias**: `forward_point_distance: 0.5` to encourage forward movement

### 2. forklift_path_planner.yaml - Cleanup
- **Removed Duplicate Controller**: Eliminated the `controller_server` section to avoid conflicts
- **Single Source of Truth**: Controller configuration now only exists in `forklift_controller.yaml`

### 3. forklift_bt_navigator.yaml - Behavior Tree Reference
- **Added Behavior Tree Path**: `default_nav_to_pose_bt_xml` pointing to `behavior.xml`
- **Ensures Proper Navigation**: BT navigator now uses the correct behavior tree

### 4. forklift_recovery.yaml - Recovery Behavior Optimization
- **Removed Spin Recovery**: Eliminated the "spin" behavior that could cause in-place rotation
- **Conservative Backup**: Reduced backup distance and speed for forklift safety
- **Faster Recovery**: Reduced wait duration for quicker response

### 5. forklift_bt_navigator.yaml - Behavior Tree Configuration
- **Added Behavior Tree Paths**: Both `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml`
- **Added Navigate Through Poses**: Included `navigate_through_poses` navigator for complete coverage
- **Ensures Proper Navigation**: BT navigator now uses the correct behavior tree for all navigation types

### 6. forklift_behavior.xml - New Forklift-Specific Behavior Tree
- **Custom Behavior Tree**: Created specifically for forklift navigation
- **No Spin Actions**: Eliminated all spinning behaviors from recovery sequence
- **Forklift-Appropriate Recovery**: Uses backup and wait instead of spinning
- **Consistent with Recovery Server**: Behavior tree matches recovery configuration

### 7. behavior.xml - Original Behavior Tree (Kept for Reference)
- **Original File Preserved**: Kept for other robots that may need spinning behavior
- **Not Used by Forklift**: Forklift now uses forklift_behavior.xml instead

## Key Parameters for Preventing In-Place Rotation

```yaml
# Angular velocity limits
max_vel_theta: 0.3
min_vel_theta: -0.3

# Critic weights prioritizing path following over rotation
PathDist.scale: 20.0
PathAlign.scale: 20.0
RotateToGoal.scale: 0.1  # Very low weight

# Rotation prevention
use_rotation_for_heading: false
angular_vel_cost: 2.0
forward_point_distance: 0.5
```

## Expected Behavior After Fix
1. **No More In-Place Rotation**: Forklift will prefer forward/backward movement with steering
2. **Better Path Following**: Prioritizes staying on the planned path
3. **Smoother Motion**: Reduced angular acceleration and better trajectory evaluation
4. **Forklift-Appropriate Recovery**: No spinning, only backing up and waiting

## Testing Recommendations
1. Test with goals that require significant heading changes
2. Verify the forklift follows paths smoothly without spinning
3. Check that recovery behaviors work without causing rotation
4. Monitor angular velocity commands to ensure they stay within limits

## Files Modified
- `noham_path_planner/config/forklift_controller.yaml`
- `noham_path_planner/config/forklift_path_planner.yaml`
- `noham_path_planner/config/forklift_bt_navigator.yaml`
- `noham_path_planner/config/forklift_recovery.yaml`
- `noham_path_planner/config/forklift_behavior.xml`
- `noham_path_planner/config/behavior.xml`
