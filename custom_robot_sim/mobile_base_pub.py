import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MobileBasePublisher(Node):

    def __init__(self):
        super().__init__('mobile_base_controller')

        self.mobile_base_publisher = self.create_publisher(
            Float64MultiArray,
            "mobile_base_controller/commands",
            10
        )

        self.mobile_base_msg = Float64MultiArray()

        # Vi matcher tidslogikken til robotarmen:
        # - ca 3 s arm går ned (30 steg à 0.1 s)
        # - ca 3 s armen står nede og venter (hold_at_pick_steps)
        self.steps_arm_down = 30        # må matche steps_per_segment i arm-koden
        self.steps_hold_at_pick = 30    # må matche hold_at_pick_steps i arm-koden

        self.cntr = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.cntr += 1

        # Fase 1: armen beveger seg ned → bilen står stille
        if self.cntr <= self.steps_arm_down:
            # Stop
            self.mobile_base_msg.data = [0.0, 0.0, 0.0, 0.0]

        # Fase 2: armen er nede og venter → bilen kjører fremover
        elif self.cntr <= self.steps_arm_down + self.steps_hold_at_pick:
            # Move Forward
            self.mobile_base_msg.data = [1.0, 1.0, 1.0, 1.0]

        # Fase 3: ferdig å vente → bilen stopper igjen
        else:
            self.mobile_base_msg.data = [0.0, 0.0, 0.0, 0.0]
            # Hvis du vil loope alt på nytt, kan du resette cntr her:
            # self.cntr = 0

        self.mobile_base_publisher.publish(self.mobile_base_msg)


def main(args=None):
    rclpy.init(args=args)
    mobile_base_node = MobileBasePublisher()
    rclpy.spin(mobile_base_node)
    mobile_base_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

