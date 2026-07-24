#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

def plan_and_execute(
    robot,
    planning_component,
    logger,
    planning_group_name,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    planning_component.set_start_state_to_current_state()
    state = planning_component.get_start_state()
    if state is not None:
        robot_model = robot.get_robot_model()
        jmg = robot_model.get_joint_model_group(planning_group_name)
        names = jmg.active_joint_model_names
        bounds = jmg.active_joint_model_bounds
        jp = dict(state.joint_positions)
        modified = False
        EPS = 1e-6
        for i, name in enumerate(names):
            joint_bounds = bounds[i]
            pos_bound = joint_bounds[0] if joint_bounds else None
            if pos_bound is None or not pos_bound.position_bounded:
                continue
            val = jp.get(name)
            if val is None:
                continue
            min_pos = pos_bound.min_position
            max_pos = pos_bound.max_position
            if val < min_pos or val > max_pos:
                jp[name] = max(min_pos + EPS, min(max_pos - EPS, val))
                modified = True
        if modified:
            state.joint_positions = jp
            planning_component.set_start_state(robot_state=state)

    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

class Controller(Node):

    def __init__(self):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10)
        self.subscription

        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        # instantiate MoveItPy instance and get planning component
        self.panda = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.logger = get_logger("moveit_py.pose_goal")

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

        self.height = 0.19
        self.pick_height_bolt = 0.113
        self.pick_height_cube = 0.133
        self.carrying_height = 0.32
        self.init_angle = -0.3825
        self.box_x = 0.3
        self.box_y = -0.3

    # function to move a gripper
    def move_to(self, x, y, z, xo, yo, zo, wo):

        self.pose_goal.pose.position.x = x
        self.pose_goal.pose.position.y = y
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = xo
        self.pose_goal.pose.orientation.y = yo
        self.pose_goal.pose.orientation.z = zo
        self.pose_goal.pose.orientation.w = wo
        self.panda_arm.set_goal_state(pose_stamped_msg = self.pose_goal, pose_link="panda_link8")
        plan_and_execute(self.panda, self.panda_arm, self.logger, "panda_arm", sleep_time=5.0)

    # function for a gripper action
    def gripper_action(self, action, object_type=0):

        self.panda_hand.set_start_state_to_current_state()

        if action == 'open':
            if object_type == 1:
                joint_values = {"panda_finger_joint1": 0.04}
            else:
                joint_values = {"panda_finger_joint1": 0.03}

        elif action == 'close':
            if object_type == 1:
                joint_values = {"panda_finger_joint1": 0.007}
            else:
                joint_values = {"panda_finger_joint1": 0.0}

        else:
            self.get_logger().info("No such action")
            return

        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state = self.robot_state,
            joint_model_group = self.panda.get_robot_model().get_joint_model_group("hand"),
        )        
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(self.panda, self.panda_hand, self.logger, "hand", sleep_time=0.5)

    def listener_callback(self, data):

        self.get_logger().info(f"{data}")

        object_type = 0
        if len(data.data) >= 4:
            object_type = int(data.data[3])

        if object_type == 1:
            pick_z = self.pick_height_cube
            self.get_logger().info("Detected: CUBE")
        else:
            pick_z = self.pick_height_bolt
            self.get_logger().info("Detected: BOLT")

        self.move_to(data.data[0], data.data[1], self.height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)

        self.gripper_action("open", object_type)

        self.move_to(data.data[0], data.data[1], pick_z, 1.0, self.init_angle + data.data[2], 0.0, 0.0)

        self.gripper_action("close", object_type)

        self.move_to(data.data[0], data.data[1], self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)

        self.move_to(self.box_x, self.box_y, self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0)

        self.gripper_action("open", object_type)

if __name__ == '__main__':
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
