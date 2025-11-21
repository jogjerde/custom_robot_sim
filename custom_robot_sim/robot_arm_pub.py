import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RobotArmPublisher(Node):

    def __init__(self):
        super().__init__('robot_arm_controller')

        self.robot_arm_publisher = self.create_publisher(
            Float64MultiArray,
            "robot_arm_controller/commands",
            10
        )
        self.create_subscription(JointState, "/joint_states", self.clbk_joint_states, 10)

        # Pose 0: armen rett opp (startpose)
        pose_up = [0.0, 0.0, math.pi/2, 0.0, 0.0]

        # Pose 1: bøyd frem, gripper peker frem (plukke)
        pose_pick = [0.0, math.pi/2, math.pi/2, -math.pi/2, 0.0]

        # Pose 2: litt opp igjen for å "bære" perlen
        pose_carry = [0.0, math.pi/3, math.pi/2, -math.pi/3, 0.0]

        self.poses = [pose_up, pose_pick, pose_carry]

        self.current_segment = 0          # 0: 0→1, 1: 1→2
        self.step_in_segment = 0
        self.steps_per_segment = 30       # ca 3 s per bevegelse (30 * 0.1 s)

        # Ventetid i plukke-posisjon (dwell time)
        self.holding_at_pick = False
        self.hold_counter = 0
        self.hold_at_pick_steps = 30      # 30 * 0.1 s = ca 3 s pause

        self.cmd_msg = Float64MultiArray()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_joint_states(self, msg):
        self.joint_names = msg.name
        self.joint_positions = msg.position
        self.joint_velocities = msg.velocity
        self.joint_efforts = msg.effort

    def timer_callback(self):
        # Hvis vi er ferdig → hold siste pose
        if self.current_segment >= len(self.poses) - 1 and not self.holding_at_pick:
            self.cmd_msg.data = self.poses[-1]
            self.robot_arm_publisher.publish(self.cmd_msg)
            return

        # PAUSE i plukkeposisjon
        if self.holding_at_pick:
            self.cmd_msg.data = self.poses[1]  # pose_pick
            self.robot_arm_publisher.publish(self.cmd_msg)

            self.hold_counter += 1
            if self.hold_counter >= self.hold_at_pick_steps:
                # Ferdig å vente → gå videre til neste segment
                self.holding_at_pick = False
                # Nå vil neste kall begynne å interpolere 1→2
            return

        # Normal interpolasjon mellom to poser
        q_start = self.poses[self.current_segment]
        q_goal = self.poses[self.current_segment + 1]

        alpha = self.step_in_segment / self.steps_per_segment

        q_cmd = []
        for qs, qg in zip(q_start, q_goal):
            q_cmd.append((1.0 - alpha) * qs + alpha * qg)

        self.cmd_msg.data = q_cmd
        self.robot_arm_publisher.publish(self.cmd_msg)

        # Oppdater steg / segment
        self.step_in_segment += 1
        if self.step_in_segment > self.steps_per_segment:
            self.step_in_segment = 0
            self.current_segment += 1

            # Hvis vi nettopp kom til plukkeposisjonen (pose 1) → start pause
            if self.current_segment == 1:
                self.holding_at_pick = True
                self.hold_counter = 0


def main(args=None):
    rclpy.init(args=args)
    robot_arm_node = RobotArmPublisher()
    rclpy.spin(robot_arm_node)
    robot_arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

