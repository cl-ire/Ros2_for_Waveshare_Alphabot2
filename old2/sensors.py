import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Int16
from ros2_for_waveshare_alphabot2.msg import ObstacleStamped, LineFollowStamped, LineFollow

class SensorDriver(Node):
    def __init__(self, dr=16, dl=19, cs=5, clock=25, address=24, dataout=23):
        super().__init__('sensor_driver')
        self.loginfo("Node 'sensors' configuring driver.")

        self.numSensors = 5
        self.DR = dr
        self.DL = dl
        self.CS = cs
        self.Clock = clock
        self.Address = address
        self.DataOut = dataout

        # Initialize the lgpio library
        self.lgpio = lgpio.gpiochip_open(0)
        
        # Set the GPIO pins as inputs with pull-up resistors
        lgpio.gpio_claim_input(self.lgpio, self.DR)
        lgpio.gpio_claim_input(self.lgpio, self.DL)
        lgpio.gpio_claim_output(self.lgpio, self.Clock)
        lgpio.gpio_claim_output(self.lgpio, self.Address)
        lgpio.gpio_claim_output(self.lgpio, self.CS)
        lgpio.gpio_claim_input(self.lgpio, self.DataOut)

        self.rate = self.create_rate(self.get_parameter_or('~rate', 10))

        # Setup publisher for obstacle detection
        self.pub_right = self.create_publisher(ObstacleStamped, 'obstacle_right', 4)
        self.pub_left = self.create_publisher(ObstacleStamped, 'obstacle_left', 4)

        # Setup publisher for the line following sensor
        self.pub_line_follow = self.create_publisher(LineFollowStamped, 'line_follow', 4)

        self.loginfo("Node 'sensors' configuration complete.")

    def __del__(self):
        # Close the lgpio library
        lgpio.gpiochip_close(self.lgpio)

    def run(self):
        DR_status = False
        DL_status = False

        self.loginfo("Node 'sensors' running.")

        while rclpy.ok():
            DR_status = not bool(lgpio.gpio_read(self.lgpio, self.DR, lgpio.PI_PUD_DOWN))
            DL_status = not bool(lgpio.gpio_read(self.lgpio, self.DL, lgpio.PI_PUD_DOWN))

            DR_message = ObstacleStamped()
            DR_message.header.stamp = self.get_clock().now().to_msg()
            DR_message.obstacle = DR_status

            DL_message = ObstacleStamped()
            DL_message.header.stamp = self.get_clock().now().to_msg()
            DL_message.obstacle = DL_status

            self.pub_right.publish(DR_message)
            self.pub_left.publish(DL_message)

            line_follow_value = self.analog_read()

            line_follow_msg = LineFollow()
            line_follow_msg.left_outer = line_follow_value[0]
            line_follow_msg.left_inner = line_follow_value[1]
            line_follow_msg.centre = line_follow_value[2]
            line_follow_msg.right_inner = line_follow_value[3]
            line_follow_msg.right_outer = line_follow_value[4]

            line_follow_stamped_msg = LineFollowStamped()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            line_follow_stamped_msg.header = header
            line_follow_stamped_msg.sensors = line_follow_msg

            self.pub_line_follow.publish(line_follow_stamped_msg)

            self.rate.sleep()

    def analog_read(self):
        value = [0] * (self.numSensors + 1)
        for j in range(0, self.numSensors + 1):
            lgpio.gpio_write(self.lgpio, self.CS, 0)
            for i in range(0, 4):
                if (((j) >> (3 - i)) & 0x01):
                    lgpio.gpio_write(self.lgpio, self.Address, 1)
                else:
                    lgpio.gpio_write(self.lgpio, self.Address, 0)
                value[j] <<= 1
                if lgpio.gpio_read(self.lgpio, self.DataOut):
                    value[j] |= 0x01
                lgpio.gpio_write(self.lgpio, self.Clock, 1)
                lgpio.gpio_write(self.lgpio, self.Clock, 0)
            for i in range(0, 6):
                value[j] <<= 1
                if lgpio.gpio_read(self.lgpio, self.DataOut):
                    value[j] |= 0x01
                lgpio.gpio_write(self.lgpio, self.Clock, 1)
                lgpio.gpio_write(self.lgpio, self.Clock, 0)
            time.sleep(0.0001)
            lgpio.gpio_write(self.lgpio, self.CS, 1)
        return value[1:]

def main():
    rclpy.init()
    node = SensorDriver()
    node.run()
    rclpy.shutdown()
    print("Node 'sensors' Stopped")

if __name__ == '__main__':
    main()
