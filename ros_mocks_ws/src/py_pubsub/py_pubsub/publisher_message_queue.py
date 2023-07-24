# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import pika
import os
import time
import logging
logging.basicConfig()


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        # ------ mq test ------
        # # --- subscription messages ---
        # url2 = os.environ.get(
        #     'CLOUDAMQP_URL', 'amqp://guest:guest@localhost:5672/%2f')
        # params2 = pika.URLParameters(url2)
        # self.connection2 = pika.BlockingConnection(params2)
        # self.sub_channel = self.connection2.channel()  # start a channel
        # self.sub_channel.queue_declare(queue='pdfprocess')  # Declare a queue
        # self.sub_channel.basic_consume('pdfprocess',
        #                                self.callback,
        #                                auto_ack=True)
        # self.sub_channel.start_consuming()

        # --- publish messages ---
        # Parse CLODUAMQP_URL (fallback to localhost)
        url = os.environ.get(
            'CLOUDAMQP_URL', 'amqp://guest:guest@localhost/%2f')
        params = pika.URLParameters(url)
        params.socket_timeout = 5

        self.connection = pika.BlockingConnection(params)  # Connect to CloudAMQP
        self.channel = self.connection.channel()  # start a channel
        self.channel.queue_declare(queue='pdfprocess')  # Declare a queue
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.rmq_pub)

    def pdf_process_function(self, msg):
        print(" PDF processing")
        print(" [x] Received " + str(msg))

        time.sleep(1)  # delays for 5 seconds
        print(" PDF processing finished")
        return

    def callback(self, ch, method, properties, body):
        print("invoking callback function")
        self.pdf_process_function(body)
        print("invoked callback function")

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    def rmq_pub(self):
        self.channel.basic_publish(
            exchange='', routing_key='pdfprocess', body='User information')
        print("[x] Message sent to consumer")

    def rmq_sub(self):
        self.channel.basic_publish(
            exchange='', routing_key='pdfprocess', body='User information')
        print("[x] Message sent to consumer")

    def __del__(self):
        print('destroy start')
        # self.connection.close()
        self.connection2.close()
        print('destroy end')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
