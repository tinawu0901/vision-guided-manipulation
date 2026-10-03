import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    MoveItErrorCodes,
)


class RobotManipulation(Node):
    def __init__(self):
        super().__init__('robot_manipulation')
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.started = False
        self.subscription = self.create_subscription(
            PoseStamped, '/detected_object_pose', self.on_pose, 10
        )
        self.get_logger().info('等待 MoveIt...')
        self.client.wait_for_server()
        self.get_logger().info('已連接 MoveIt，等待物件位置。')

    def on_pose(self, pose):
        if self.started:
            return
        if pose.header.frame_id != 'panda_link0':
            self.get_logger().error('目前只接受 panda_link0 座標。')
            return
        self.started = True

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'panda_arm'
        request.start_state.is_diff = True
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        position = PositionConstraint()
        position.header.frame_id = pose.header.frame_id
        position.link_name = 'panda_hand'
        position.weight = 1.0

        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.01]
        position.constraint_region.primitives.append(region)

        region_pose = PoseStamped().pose
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitive_poses.append(region_pose)

        orientation = OrientationConstraint()
        orientation.header.frame_id = pose.header.frame_id
        orientation.link_name = 'panda_hand'
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 0.1
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info('收到位置，開始規劃並執行...')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal)

    def on_goal(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error('MoveIt 拒絕目標。重啟程式後再試。')
                return
            self.get_logger().info('MoveIt 接受目標，等待執行結果...')
            handle.get_result_async().add_done_callback(self.on_result)
        except Exception as error:
            self.get_logger().error(str(error))

    def on_result(self, future):
        try:
            code = future.result().result.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('成功！Panda 已到達目標。')
            else:
                self.get_logger().error(f'MoveIt 失敗，錯誤碼：{code}')
        except Exception as error:
            self.get_logger().error(str(error))


def main():
    rclpy.init()
    node = RobotManipulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
