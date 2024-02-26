#!/usr/bin/python3

from cl2703.task3b import get_package_config
import rclpy
from threading import Thread
from time import sleep
from laser_utils import LaserToImg
from task1a import aruco_tf

from cl2703.flags import ARENA
if ARENA:
    from cl2703.task4a import RackShiftHW as RackShift
    from cl2703.task4b import PickAndDrop as PickAndDrop
else:
    from cl2703.task2b import RackShift
    from cl2703.task2a import PickAndDrop

### This is the final task script, will be used Task 6 onwards ###
# Design/control flow:
#
# * Design preferences:
#     - Minimum rewriting of old scripts. As much as possible, new code
#       will be in this script only.
#     - No background scripts/additional launches (e.g. docker, TF detector, etc.)
#
# * Nodes used in this script:
#     - arm_node: picking and dropping boxes
#     - nav_node: picking and dropping racks
#     - aruco_node: aruco marker detection
#     - dock_node: docking to the rack
# * Each node (except dock_node) has one thread controlling its actions. These threads
#   run in the rclpy executor, and are be orchestrated by the task configuration list, `task_info`.
#
# * Ez buddy, let's work on arm shoulder angle preemption a little later
###

rclpy.init ()
executor = rclpy.executors.MultiThreadedExecutor (10) # 4 nodes, 3 control threads + some buffer


# Task instructions. In addition to rack/box info itself,
# threads use the contents of each element as cues for their operations.
# The following signals are currently in use:
# * task_info [t]['rack'] is None  ->  rack has been shifted
# * task_info [t]['box']  is None  ->  box has been identified from camera feed
task_info = get_package_config ('config.yaml')


### Nodes ###
arm_node   = PickAndDrop ("arm_control")
nav_node   = RackShift ()
aruco_node = aruco_tf ()
dock_node  = LaserToImg ()

aruco_node.destroy_timer (aruco_node.timer) # Don't keep hogging resources
for n in (arm_node, nav_node, aruco_node, dock_node):
    executor.add_node (n)


### Thread callbacks ###
# Each of these callbacks handles the working of one stage of the task.
# They use the task configuration list (`task_info`) for synchronization.
#
# For example, after the rack of task_info [t] is shifted,
# task_info [t]['rack'] in is cleared to `None`. This is the cue for the
# arm to start picking the box of task_info [t].
def nav_th_fn ():
    # eBot navigation
    # home pose; dock at rack; drop rack at arm
    for t in range (len (task_info)):
        orig = task_info [t]['pickup']['trans']; dest = task_info [t]['drop']['trans']
        if ((orig[0] - dest[0])**2 + (orig[1] - dest[1])**2)**0.5 > 0.8:
            # Rack needs shifting
            nav_node.rack_shift (task_info [t], dock_node)
        task_info [t]['rack'] = None
#executor.create_task (nav_th_fn)
Thread (target = nav_th_fn).start ()

def aruco_th_fn ():
    # Aruco detection
    # wait for rack to arrive at arm; analyze camera feed; stop when arm destination is set
    for t in range (len (task_info)):
        while task_info [t]['rack'] is not None: sleep (0.5) # Wait for the rack to arrive
        while task_info [t]['box'] is not None: # Work until arm_node knows the coordinates of its next box
            aruco_node.process_image ()
#executor.create_task (aruco_th_fn)
Thread (target = aruco_th_fn).start ()

def arm_th_fn ():
    # UR5 Arm control
    # wait for rack to arrive at arm; identify boxes; pick boxes
    # Additionally, set the signal for the executor to stop
    for t in range (len (task_info)):
        while task_info [t]['rack'] is not None: sleep (0.5) # Wait for the rack to arrive
        arm_node.pick_and_drop ([ task_info [t]['box'] ]) # Box identification
        task_info [t]['box'] = None
        arm_node.motion ()
    task_future.set_result (0)
#task_future = executor.create_task (arm_th_fn)
task_future = rclpy.task.Future ()
Thread (target = arm_th_fn).start ()

executor.spin_until_future_complete (task_future)

rclpy.shutdown ()
exit ()
