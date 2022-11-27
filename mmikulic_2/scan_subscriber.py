import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

goal_position = Point()
ray_angle = 0.0
min_distance = 0.0
current_position = Point()
current_yaw = 0.0
Kp = 0.2
linear_speed_const = 0.07
goto_closest = False

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')
        self.goto_closest_srv = self.create_service(Empty, 'goto_closest', self.goto_closest_callback)
        self.create_subscription(Odometry, '/odom', self.current_position_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.publisher_marker = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)

    def goto_closest_callback(self, request, response):
        global goto_closest
        self.get_logger().info(f'Got service call - goto_closest')
        goto_closest = True
        return response

    def scan_callback(self, laser_scan):
        global min_distance, ray_angle
        ranges = laser_scan.ranges
        min_distance = min(ranges)

        if min_distance < 100 and min_distance > 0:
            ray_angle = ranges.index(min_distance) * laser_scan.angle_increment

            if goto_closest == True:
                self.move_to_goal()

    def current_position_callback(self, odom):
        global current_position, current_yaw
        current_position = odom.pose.pose.position
        orientation_quat = odom.pose.pose.orientation
        r =  R.from_quat([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        current_yaw = r.as_euler('xyz', degrees=False)[2]

        goal_position.x = current_position.x + min_distance * math.cos(ray_angle + current_yaw)
        goal_position.y = current_position.y + min_distance * math.sin(ray_angle + current_yaw)
        goal_position.z = 0.

        if min_distance != 0.:
            self.place_marker()


    def move_to_goal(self):
        global goto_closest
        delta_yaw = self.calculate_delta_yaw(goal_position.y, current_position.y, goal_position.x, current_position.x)
        
        goal_angle = delta_yaw - current_yaw
        goal_delta_x = goal_position.x - current_position.x
        goal_delta_y = goal_position.y - current_position.y

        if goal_angle > math.pi:
            goal_angle -= 2*math.pi
        if goal_angle <= -math.pi:
            goal_angle += 2*math.pi


        command = Twist()
        if not 0 <= abs(goal_angle) < 0.01:            
            command.angular = Vector3(x=0.0, y=0.0, z=Kp*goal_angle)
        else:
            command.angular = Vector3(x=0.0, y=0.0, z=0.0)

        if 0 <= abs(goal_delta_x) < 0.31 and 0 <= abs(goal_delta_y) < 0.31:
            command.linear = Vector3(x=0.0, y=0.0, z=0.0)
            command.angular = Vector3(x=0.0, y=0.0, z=0.0)
            goto_closest = False
        else:
            if abs(goal_angle) > math.pi/6.:
                command.linear = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                command.linear = self.adjust_linear_speed(goal_delta_x, goal_delta_y)
        
        self.publisher_cmd.publish(command)

    def adjust_linear_speed(self, goal_delta_x, goal_delta_y):
        if 0 <= abs(goal_delta_x) < 0.6 and 0 <= abs(goal_delta_y) < 0.6:
            return Vector3(x=linear_speed_const/2., y=0.0, z=0.0)
        else:
            return Vector3(x=linear_speed_const, y=0.0, z=0.0)

    def calculate_delta_yaw(self, goal_y, current_y, goal_x, current_x):
       return math.atan2(goal_y - current_y, goal_x - current_x)

    def place_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = 3
        marker.id = 0
        marker.action = Marker.ADD

        marker.pose.position.x = goal_position.x
        marker.pose.position.y = goal_position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.a = 0.8
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 0.9
        self.publisher_marker.publish(marker)

        

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber =  ScanSubscriber()
    rclpy.spin(scan_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()