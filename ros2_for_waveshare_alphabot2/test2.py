import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray, Float32, Float32MultiArray
from geometry_msgs.msg import Twist



class MovementTest(Node):
    def __init__(self):
        super().__init__('movement_test')
        #create the subscriber 
        self.subscription = self.create_subscription(
            String,                     #data type
            'joystick',                 #topic published by camera_opencv_node
            self.listener_callback,     #function to notify that a mesage was recived
            5)                          #queue size amount of the stored mesages  
        self.subscription

        # Message publishers
        self.servo_pub = self.create_publisher(Int32MultiArray, '/Servo', 4)



    def listener_callback(self, msg):
        # Messages
        imput = msg.data
        self.get_logger().info("Imput recived: {}".format(imput)) 
        
        servo_msg = [0, 0]
        last_servo_msg = [0, 0]

        if imput == String(data="Center"):
            servo_msg = [0, 0]
        elif imput == String(data="Up"):
            servo_msg = [70, 0]
        elif imput == String(data="Right"):
            servo_msg = [0, 90]
        elif imput == String(data="Left"):
            servo_msg = [0, -90]
        elif imput == String(data="Down"):
            servo_msg = [-20, 0]

        servo_msg_sent = Int32MultiArray()
        
        if servo_msg != last_servo_msg:
            self.get_logger().info("Data sent to Servo: {}".format(servo_msg))
            servo_msg_sent.data = servo_msg
            self.servo_pub.publish(servo_msg_sent)
            last_servo_msg = servo_msg
                
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
