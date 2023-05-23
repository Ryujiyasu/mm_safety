import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

NORMAL = 0
SLOW = 1
STOP = 2

PWM = 0
AUTO = 1
MANUAL = 2


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85036313430351901210-if00', 9600, timeout=None)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.flash_timer = self.create_timer(1, self.flash_timer_callback)
        self.status = NORMAL
        self.status_count = 0
        self.mm_status = PWM
        self.stop_num = None
        self.stop_distance = None
        self.publisher_ = self.create_publisher(Twist, 'post_cmd_vel', 10)
        self.mm_status_publisher = self.create_publisher(Int8, 'mm_flash', 10)
        self.mm_status_subscription = self.create_subscription(
            Int8,
            'mm_status',
            self.status_listener_callback,
            10)
        self.mm_reset_subscription = self.create_subscription(
            Int8,
            'mm_reset',
            self.reset_listener_callback,
            10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def __del__(self):
        self.ser.close()

    def reset_listener_callback(self):
        print("reset")
        self.status = NORMAL

    def flash_timer_callback(self):
        msg = Int8()
        msg.data = self.status
        self.mm_status_publisher.publish(msg)

    def status_listener_callback(self, msg):
        self.mm_status = msg.data

    def listener_callback(self, msg):
        if self.mm_status == MANUAL:
            if self.status == NORMAL:
                self.publisher_.publish(msg)
            elif self.status == SLOW:
                msg.linear.x = msg.linear.x / 2
                msg.angular.z = msg.angular.z/2
                self.publisher_.publish(msg)
            elif self.status == STOP:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
        elif self.mm_status == AUTO:
            if self.status == NORMAL:
                self.publisher_.publish(msg)
            elif self.status == SLOW:
                msg.linear.x = msg.linear.x
                msg.angular.z = msg.angular.z
                self.publisher_.publish(msg)
            elif self.status == STOP:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
        else:
            if self.status == NORMAL:
                self.publisher_.publish(msg)
            elif self.status == SLOW:
                msg.linear.x = msg.linear.x
                msg.angular.z = msg.angular.z
                self.publisher_.publish(msg)
            elif self.status == STOP:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
            

    def timer_callback(self):
        data = self.ser.readline()
        try:
            data = str(data.decode('utf-8')).split(',')
            number = data[0]
            distance = data[1]
            if float(distance) != 0.0:
                if (number == '1' or number == '5') and float(distance) <= 50:
                    self.status = STOP
                    self.stop_num = number
                    self.stop_distance = distance
                elif (number == '2' or number == '6') and float(distance) <= 20:
                    self.status = STOP
                    self.stop_num = number
                    self.stop_distance = distance
                elif (number == '3' or number == '7') and float(distance) <= 50:
                    self.status = STOP
                    self.stop_num = number
                    self.stop_distance = distance
                elif (number == '4' or number == '8') and float(distance) <= 20:
                    self.status = STOP
                    self.stop_num = number
                    self.stop_distance = distance
                if self.status != STOP:
                    if (number == '1' or number == '5') and float(distance) <= 100:
                        self.status = SLOW
                        self.status_count = 0
                        self.stop_num = number
                        self.stop_distance = distance
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.publisher_.publish(msg)
                    elif (number == '2' or number == '6') and float(distance) <= 40:
                        self.status = SLOW
                        self.status_count = 0
                        self.stop_num = number
                        self.stop_distance = distance
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.publisher_.publish(msg)
                    elif (number == '3' or number == '7') and float(distance) <= 100:
                        self.status = SLOW
                        self.status_count = 0
                        self.stop_num = number
                        self.stop_distance = distance
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.publisher_.publish(msg)
                    elif (number == '4' or number == '8') and float(distance) <= 40:
                        self.status = SLOW
                        self.status_count = 0
                        self.stop_num = number
                        self.stop_distance = distance
                        msg = Twist()
                        msg.angular.z = 0.0
                        msg.linear.x = 0.0
                        self.publisher_.publish(msg)
                    else:
                        self.status_count += 1

                    if self.status_count >= 30:
                        self.status = NORMAL
                else:
                    if (number == '1' or number == '5') and float(distance) <= 100:
                        pass
                    elif (number == '2' or number == '6') and float(distance) <= 40:
                        pass
                    elif (number == '3' or number == '7') and float(distance) <= 50:
                        pass
                    elif (number == '4' or number == '8') and float(distance) <= 40:
                        pass
                    else:
                        self.status_count += 1
                        
                    if self.status_count >= 100:
                        self.status = NORMAL

            print(self.status, self.status_count, self.stop_num, self.stop_distance)

        except Exception as e:
            print(e)
        if self.status == STOP:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
