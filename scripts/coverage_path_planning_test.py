#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

class CoveragePathPlanner:
    def __init__(self):
        rospy.init_node('coverage_path_planner')
        
        # Configuration parameters
        self.THRESHOLD = 0.25
        self.ORIGIN_X = 25
        self.ORIGIN_Y = 25
        self.GRID_RESOLUTION = 0.4
        
        # State variables
        self.current_map = None
        self.robot_pose = Pose()
        self.map_received = False
        self.odom_received = False 
        
        # Setup ROS components
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.move_base = SimpleActionClient('move_base', MoveBaseAction)

        # Wait for map
        while not rospy.is_shutdown() and self.current_map is None:
            rospy.sleep(0.1)
        
        # Wait for move_base
        self.move_base.wait_for_server()

        # Launch coverage planner
        self.run_coverage_planner()

    
    def run_coverage_planner(self):
        """
        Main execution flow with proper error handling
        """
        # Process map and plan path
        reduced_map = self.process_map()
        coverage_path = self.plan_path(reduced_map)

        # Execute path
        self.execute_path(coverage_path, reduced_map)
        rospy.loginfo("Coverage complete!")
    

    def process_map(self):
        """
        Process occupancy grid into simplified map with error handling
        """
        resolution_ratio = self.GRID_RESOLUTION / self.current_map.info.resolution
        grid_ratio = int(round(resolution_ratio))
        
        # Reshape and reduce resolution
        original_height = self.current_map.info.height
        original_width = self.current_map.info.width
        original_data = np.array(self.current_map.data).reshape(original_height, original_width)
        
        # Create reduced map by checking blocks
        reduced_map = np.zeros((original_height // grid_ratio, original_width // grid_ratio), dtype=np.int8)
        
        for i in range(0, original_height, grid_ratio):
            for j in range(0, original_width, grid_ratio):
                block = original_data[i:i+grid_ratio, j:j+grid_ratio]

                # If any obstacle in the block
                if np.any(block > 0): 
                    reduced_map[i//grid_ratio, j//grid_ratio] = 1
                    
        return reduced_map


    def plan_path(self, map_data):
        """
        Generate coverage path using BFS with weights
        """
        # Initialize grid
        height, width = map_data.shape
        grid = np.zeros((height, width), dtype=[('x', int),       ('y', int),
                                                 ('weight', int), ('visited', bool)])
        
        # Initialize grid cells
        for i in range(height):
            for j in range(width):
                grid[i,j] = (i, j, 0, bool(map_data[i,j]))
        
        # BFS from start position
        start_x, start_y = self.ORIGIN_X, self.ORIGIN_Y
        queue = [(start_x, start_y)]
        grid[start_x, start_y]['weight'] = 1
        
        # Assign weights using BFS
        while queue:
            x, y = queue.pop(0)
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < height and 0 <= ny < width:
                    if not grid[nx, ny]['visited'] and grid[nx, ny]['weight'] == 0:
                        grid[nx, ny]['weight'] = grid[x, y]['weight'] + 1
                        queue.append((nx, ny))
        
        # Generate coverage path
        path = []
        current = grid[start_x, start_y]
        current['visited'] = True
        path.append(current)
        
        while True:
            # Find adjacent unvisited cell with max weight
            next_cell = None
            max_weight = -1
            x, y = current['x'], current['y']
            
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < height and 0 <= ny < width:
                    if not grid[nx, ny]['visited'] and grid[nx, ny]['weight'] > max_weight:
                        max_weight = grid[nx, ny]['weight']
                        next_cell = grid[nx, ny]
            
            if next_cell is None:
                # Find closest unvisited cell
                min_dist = float('inf')
                for i in range(height):
                    for j in range(width):
                        if not grid[i,j]['visited']:
                            dist = np.hypot(i-current['x'], j-current['y'])
                            if dist < min_dist:
                                min_dist = dist
                                next_cell = grid[i,j]
                if next_cell is None:
                    break  # All cells visited
            
            next_cell['visited'] = True
            path.append(next_cell)
            current = next_cell
        
        rospy.loginfo(f"Generated path with {len(path)} waypoints")
        return path
    

    def execute_path(self, path, map_data):
        """
        Execute the planned path with navigation
        """
        for i, waypoint in enumerate(path):
            if rospy.is_shutdown():
                break

            # Create navigation goal
            goal = self.create_nav_goal(waypoint)
            rospy.loginfo(f"Navigating to [{waypoint['x']},{waypoint['y']}] at ({goal.target_pose.pose.position.x:.2f},{goal.target_pose.pose.position.y:.2f})")
            
            # Reaching waypoint with retries
            for attempt in range(3):
                if self.navigate_to_goal(goal):
                    break

    
    def create_nav_goal(self, waypoint):
        """
        Create a navigation goal with proper orientation
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Convert grid to world coordinates
        goal.target_pose.pose.position.x = (waypoint['y'] - self.ORIGIN_Y) * self.GRID_RESOLUTION
        goal.target_pose.pose.position.y = (waypoint['x'] - self.ORIGIN_X) * self.GRID_RESOLUTION
        
        # Calculate orientation towards goal
        dx = goal.target_pose.pose.position.x - self.robot_pose.position.x
        dy = goal.target_pose.pose.position.y - self.robot_pose.position.y
        yaw = np.arctan2(dy, dx)
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        return goal
    

    def navigate_to_goal(self, goal):
        """
        Execute navigation to a single waypoint
        """
        self.move_base.send_goal(goal)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Check if reached
            distance = np.hypot(
                self.robot_pose.position.x - goal.target_pose.pose.position.x,
                self.robot_pose.position.y - goal.target_pose.pose.position.y)
            
            if distance < self.THRESHOLD:
                rospy.loginfo("Waypoint reached")
                return True
            
            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > 30.0:
                rospy.logwarn("Navigation timeout")
                self.move_base.cancel_goal()
                return False
            
            # Check for failures
            state = self.move_base.get_state()
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
                rospy.logwarn(f"Navigation failed with state {GoalStatus.to_string(state)}")
                return False
            
            rospy.sleep(0.1)
        
        return False
    

    def map_callback(self, msg):
        self.current_map = msg
        self.map_received = True
        rospy.loginfo_once("Received first map")
    

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.odom_received = True


if __name__ == '__main__':
    try:
        CoveragePathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stopped")
    except Exception as e:
        rospy.logerr(f"Fatal error: {str(e)}")