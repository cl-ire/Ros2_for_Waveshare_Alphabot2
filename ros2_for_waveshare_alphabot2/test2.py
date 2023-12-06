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
        self.pantilt_pub = self.create_publisher(Float32MultiArray, 'pan_tilt', 4)



    def listener_callback(self, msg):
        # Messages
        pantilt_msg = Float32MultiArray()
        imput = msg.data
        self.get_logger().info("Imput recived: {}".format(imput)) 
        

        if imput == String(data="Center"):
            pantilt_msg.data = [0.0, 0.0]
        elif imput == String(data="Up"):
            pantilt_msg.data = [0.0, 1.0]
            
        elif imput == String(data="Right"):
            pantilt_msg.data = [1.0, 0.0]
            
        elif imput == String(data="Left"):
            pantilt_msg.data = [-1.0, 0.0]
            
        elif imput == String(data="Down"):
            pantilt_msg.data = [0.0, -1.0]
            

        self.pantilt_pub.publish(pantilt_msg)
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
