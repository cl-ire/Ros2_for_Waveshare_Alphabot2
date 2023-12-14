import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import time


class MovementTest(Node):
    def __init__(self):
        super().__init__('movement_test')
        #create the subscriber 
        self.subscription = self.create_subscription(
            String,                     #data type
            '/joystick',                 #topic published by camera_opencv_node
            self.listener_callback,     #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription

        self.servo_msg_hold = [0, 0]
        # Message publishers
        self.servo_pub = self.create_publisher(Int32MultiArray, '/servo', 4)
    

    def listener_callback(self, msg):
        # Messages
        imput = msg.data
        self.get_logger().info("Imput recived: {}".format(imput)) 
        
        # channel 0 = pan 
		# channel 1 = tilt
        

        if imput == "Center":
            self.servo_msg_hold = [0, 0]
            servo_msg = [0, 0]
        elif imput == "Up":
            servo_msg = [0, 5]
        elif imput == "Right":
            servo_msg = [5, 0]
        elif imput == "Left":
            servo_msg = [-5, 0]
        elif imput == "Down":
            servo_msg = [0, -5]

        self.servo_msg_hold[0] = self.servo_msg_hold[0] + servo_msg[0]
        self.servo_msg_hold[1] = self.servo_msg_hold[1] + servo_msg[1]

        servo_msg_sent = Int32MultiArray()
        
        self.get_logger().info("Data sent to Servo: {}".format(self.servo_msg_hold))
        servo_msg_sent.data = self.servo_msg_hold
        self.servo_pub.publish(servo_msg_sent)
                        
        time.sleep(0.2)


def main(args=None):
    
    rclpy.init()
    movement_test = MovementTest()
    rclpy.spin(movement_test)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    movement_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
