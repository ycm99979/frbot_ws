#!/usr/bin/env python3
"""
============================================================================
Visual Servo Controller Node
============================================================================

/target_point (geometry_msgs/PointStamped) 토픽을 구독하여
엔드이펙터가 해당 좌표를 추종하도록 MoveIt Servo에 Twist 명령 전송

[구독 토픽]
- /target_point (geometry_msgs/PointStamped): 추종할 목표 좌표

[발행 토픽]  
- /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped): Servo 명령

[파라미터]
- servo_gain: 비례 제어 게인 (default: 1.0)
- max_linear_vel: 최대 선속도 (default: 0.2 m/s)
- position_tolerance: 위치 허용 오차 (default: 0.01 m)
- control_rate: 제어 주기 (default: 50 Hz)

============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped, TwistStamped
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # Required for PointStamped transform support

import numpy as np
from threading import Lock


class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')
        
        # Parameters
        self.declare_parameter('servo_gain', 1.0)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('target_topic', 'target_point')
        self.declare_parameter('planning_frame', 'arm_base_link')
        self.declare_parameter('ee_frame', 'end_effector_link')
        self.declare_parameter('auto_enable', True)
        self.declare_parameter('servo_enable_service', '/servo_node/start_servo')
        
        self.servo_gain = self.get_parameter('servo_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.target_topic = self.get_parameter('target_topic').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.auto_enable = self.get_parameter('auto_enable').value
        self.servo_enable_service = self.get_parameter('servo_enable_service').value
        
        # Callback group for concurrent execution
        self.cb_group = ReentrantCallbackGroup()
        
        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Target point storage
        self.target_point = None
        self.target_lock = Lock()
        self.servo_enabled = False
        
        # Subscriber: target point from YOLO or other source
        # Make topic configurable because some packages publish as 'point_target' etc.
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Publisher: twist commands to MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Service: enable/disable servo
        self.enable_srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback,
            callback_group=self.cb_group
        )

        # Optional client to MoveIt Servo start service (Trigger type)
        self.servo_enable_client = self.create_client(Trigger, self.servo_enable_service)
        
        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
            callback_group=self.cb_group
        )
        
        self.get_logger().info('Visual Servo Controller initialized')
        self.get_logger().info(f'  - Servo gain: {self.servo_gain}')
        self.get_logger().info(f'  - Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  - Position tolerance: {self.position_tolerance} m')
        self.get_logger().info(f'  - Control rate: {self.control_rate} Hz')
        self.get_logger().info('Call ~/enable service with data:=true to start servoing')

        # Auto-enable behavior: enable internal flag and optionally call MoveIt Servo service
        if self.auto_enable:
            self.get_logger().info('Auto-enable requested: enabling visual servo controller')
            # Set internal flag so control loop runs
            self.servo_enabled = True
            # Try to call external servo enable service with retries (service may start later)
            self._enable_attempts = 0
            self._max_enable_attempts = 10
            self._enable_timer = self.create_timer(1.0, self._attempt_enable_service)
            # Also try immediately (non-blocking)
            self._call_servo_enable_service(True)

    def _attempt_enable_service(self):
        """Timer callback: try to call servo enable service until success or attempts exhausted."""
        # If already attempted enough times, stop
        if self._enable_attempts >= self._max_enable_attempts:
            try:
                self.destroy_timer(self._enable_timer)
            except Exception:
                pass
            self.get_logger().warn('Servo enable attempts exhausted')
            return

        # If service is available, call it and stop timer
        if self.servo_enable_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'{self.servo_enable_service} available, calling enable')
            self._call_servo_enable_service(True)
            try:
                self.destroy_timer(self._enable_timer)
            except Exception:
                pass
            return

        self._enable_attempts += 1
        self.get_logger().debug(f'Waiting for servo enable service... attempt {self._enable_attempts}')

    def _call_servo_enable_service(self, enable: bool):
        """Call MoveIt Servo's start_servo service (Trigger type)."""
        if not enable:
            return  # Only start, no stop for now

        req = Trigger.Request()

        # Wait briefly for the service to appear
        if self.servo_enable_client.wait_for_service(timeout_sec=2.0):
            fut = self.servo_enable_client.call_async(req)
        else:
            self.get_logger().warn(f'Servo start service {self.servo_enable_service} not available')
            return

        def _done(future):
            try:
                res = future.result()
                if res.success:
                    self.get_logger().info(f'MoveIt Servo started: {res.message}')
                else:
                    self.get_logger().warn(f'MoveIt Servo start failed: {res.message}')
            except Exception as e:
                self.get_logger().warn(f'Error calling servo start service: {e}')

        fut.add_done_callback(_done)

    def target_callback(self, msg: PointStamped):
        """목표점 수신 콜백"""
        with self.target_lock:
            self.target_point = msg
        self.get_logger().debug(f'Received target: header.frame_id={msg.header.frame_id} point=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})')
        
    def enable_callback(self, request, response):
        """서보 활성화/비활성화 서비스"""
        self.servo_enabled = request.data
        response.success = True
        response.message = f"Visual servo {'enabled' if self.servo_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response
    
    def get_current_ee_position(self):
        """현재 엔드이펙터 위치를 planning frame 기준으로 반환"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
    
    def transform_target_to_planning_frame(self, target: PointStamped):
        """목표점을 planning frame으로 변환"""
        # If already in planning frame, return directly
        if target.header.frame_id == self.planning_frame:
            return np.array([target.point.x, target.point.y, target.point.z])

        try:
            # Use TF2 to properly transform the PointStamped (handles rotation + translation)
            transformed = self.tf_buffer.transform(
                target,
                self.planning_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            return np.array([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            ])

        except Exception as e:
            # Could be TransformException or other errors from tf2
            self.get_logger().warn(f'Target transform failed: {e}')
            return None
    
    def control_loop(self):
        """메인 제어 루프 - 목표점 추종"""
        if not self.servo_enabled:
            return
        
        # Get target point
        with self.target_lock:
            if self.target_point is None:
                return
            target = self.target_point
        
        # Get current EE position
        current_pos = self.get_current_ee_position()
        if current_pos is None:
            return
        
        # Transform target to planning frame
        target_pos = self.transform_target_to_planning_frame(target)
        if target_pos is None:
            return
        
        # Calculate position error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)
        
        # Check if within tolerance
        if error_norm < self.position_tolerance:
            # Send zero velocity
            self.publish_twist(0.0, 0.0, 0.0)
            return
        
        # Proportional control with velocity limiting
        velocity = self.servo_gain * error
        vel_norm = np.linalg.norm(velocity)
        
        if vel_norm > self.max_linear_vel:
            velocity = velocity * (self.max_linear_vel / vel_norm)
        
        # Publish twist command
        self.publish_twist(velocity[0], velocity[1], velocity[2])
    
    def publish_twist(self, vx: float, vy: float, vz: float):
        """TwistStamped 메시지 발행"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        # Use end-effector frame for twist header (MoveIt Servo expects commands in EE frame)
        twist_msg.header.frame_id = self.ee_frame
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        
        # No rotation for now (position-only tracking)
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        
        self.twist_pub.publish(twist_msg)
        # Log non-zero commands at INFO so users see activity without debug mode
        if abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(vz) > 1e-6:
            self.get_logger().info(f'Published twist -> frame: {twist_msg.header.frame_id} linear=({vx:.3f},{vy:.3f},{vz:.3f})')
        else:
            self.get_logger().debug(f'Published zero twist in frame {twist_msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    
    node = VisualServoController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
