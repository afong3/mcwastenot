import rclpy
from rclpy.node import Node
import serial
import threading
from geometry_msgs.msg import Twist  # or any other relevant message type

# Import ODrive functions
from .odrive_control import connect_odrive, calibrate_motor, setup_open_loop, set_velocity, stop_motor

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.odrv = connect_odrive()
        if self.odrv is not None:
            calibrate_motor(self.odrv)
            setup_open_loop(self.odrv)

        # Initialize subscriber to MoveIt 2 topic
        self.subscription = self.create_subscription(
            Twist,  # Change this type based on your specific requirement
            'topic_name',  # Change to the name of the topic used by MoveIt 2
            self.moveit_callback,
            10
        )
        self.encoder_thread = threading.Thread(target=self.read_encoder_values)
        self.encoder_thread.start()

    def moveit_callback(self, msg):
        # Handle incoming message from MoveIt 2
        # For example, extract target position or velocity
        target_velocity = msg.linear.x  # adjust according to message type
        # Perform control based on target_velocity

    def read_encoder_values(self):
        try:
            with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
                while rclpy.ok():
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').rstrip()
                        # Parse line for encoder values
                        encoder_value = float(line)  # Adjust this according to your data format
                        self.control_loop(encoder_value)
        except Exception as e:
            self.get_logger().error('Error in reading encoder values: ' + str(e))


    def control_loop(self, encoder_value, target_velocity):
        # closed-loop control logic here
        # ...

    # Other methods...

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
