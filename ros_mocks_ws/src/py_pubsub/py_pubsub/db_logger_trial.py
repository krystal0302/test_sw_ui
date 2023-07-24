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

import sqlite3
import uuid
from datetime import datetime

# from std_msgs.msg import String
# from far_fleet_msgs.msg import FleetState
from far_fleet_msgs.msg import Event
from far_fleet_msgs.msg import FleetState
from far_plan_msgs.msg import FlowState
from far_plan_msgs.msg import FlowSummary

import sqlite3
from sqlite3 import Error
# import redis

logDB = '/home/farobot/far_app_data/app_log/test_log/systemlog.db'
fleetStateDB = '/home/farobot/far_app_data/app_log/test_log/fleetState.db'
flowStateDB = '/home/farobot/far_app_data/app_log/test_log/flowState.db'

systemLogTable = '''CREATE TABLE IF NOT EXISTS syslog(
                        TIME        TEXT NOT NULL, 
                        event_type  TEXT NOT NULL, 
                        event_id    TEXT PRIMARY KEY NOT NULL, 
                        module_pub  INT  NOT NULL, 
                        module_sub  TEXT, 
                        level       TEXT, 
                        msg_cont    TEXT, 
                        robot_id    TEXT );'''

fleetStateTable = '''CREATE TABLE IF NOT EXISTS FleetState(
                        ts               TEXT NOT NULL, 
                        robot_id         TEXT PRIMARY KEY NOT NULL, 
                        robot_name       TEXT, 
                        model            TEXT, 
                        mode             INT, 
                        role             TEXT, 
                        pose_x           REAL, 
                        pose_y           REAL, 
                        pose_a           REAL, 
                        mileage          REAL, 
                        battery_percent  REAL, 
                        fleet_name       TEXT, 
                        last_updated     TEXT );'''

flowStateTable = '''CREATE TABLE IF NOT EXISTS FlowState(
                        ts               TEXT NOT NULL, 
                        flow_id          TEXT PRIMARY KEY NOT NULL, 
                        flow_name        TEXT, 
                        state            INT, 
                        progress         INT, 
                        fleet_name       TEXT, 
                        last_updated     TEXT NOT NULL);'''                        

class Logger(Node):

    def __init__(self):
        super().__init__('system_logger')

        ### Database Initialization
        self.syslogConn = self.create_connection(logDB)
        if self.syslogConn is not None:
            self.create_table(self.syslogConn, systemLogTable)
        else:
            print("Error! Cannot create System Log Database Connection")

        self.fltsttConn = self.create_connection(fleetStateDB)
        if self.fltsttConn is not None:
            self.create_table(self.fltsttConn, fleetStateTable)
        else:
            print("Error! Cannot create Fleet State Database Connection")

        self.flwsttConn = self.create_connection(flowStateDB)
        if self.flwsttConn is not None:
            self.create_table(self.flwsttConn, flowStateTable)
        else:
            print("Error! Cannot create Flow State Database Connection")

        ### Event subscription
        self.event_sub_ = self.create_subscription(Event, 'event', self.systemLogCallback, 10)

        ### fleet-state subscription
        self.fleet_state_sub_ = self.create_subscription(FleetState, 'fleet_state', self.fleetStateCallback, 10)

        ### flow-state subscription
        self.flow_state_sub_ = self.create_subscription(FlowState, 'flow_state', self.flowStateCallback, 10)

        ### subscriber
        self.subscription = self.create_subscription( FlowState, 'flow_state', self.listener_callback, 10)
        # self.subscription  # prevent unused variable warning


        ### publisher
        self.publisher_ = self.create_publisher(FlowState , 'flow_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # Redis connection
        # self.r = redis.Redis(host='localhost', port=6379, decode_responses=True)

    def systemLogCallback(self, msg):
        timeNow = str(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        eventUUID = str(uuid.uuid4()).replace("-", "")
        print(timeNow)
        print(eventUUID)

    def fleetStateCallback(self, msg):
        print(str(msg))

    def flowStateCallback(self, msg):
        # print(str(msg))
        timeNow = str(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        eventUUID = str(uuid.uuid4()).replace("-", "")
        print(timeNow)
        print(eventUUID)
        event = ()
        self.syslogConn.execute("INSERT INFO")


    def listener_callback(self, msg):
        # print(msg)
        # self.get_logger().info('I heard: messages from flow_state')
        # jsonData = """{ flow_id: {flowId} } """.format(flowId =msg.flow_state.flow_id)
        print('--- listen to the flow state message ---')
        print(str(msg))

        # print(self.r.ping())
        # data = """{'name': 'bot', \
        #         'age': 25, \
        #         'goods': ['artifact', 'mbp'], \
        #         'detail': {'address': 'AUO', 'job': 'Move'}})"""
        # self.r.set('flow_state', data);
        # self.r.set('flow_state', str(msg))

    def timer_callback(self):
        submsg = FlowSummary()
        submsg.flow_id = 'flow_id_1'
        submsg.flow_name = 'flow_name_1'
        submsg.fleet_name = 'fleet_name_1'
        submsg.state = 0 
        submsg.complete_percent = 60.8 

        msg = FlowState()
        msg.flow_state.append(submsg)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def create_connection(self, db_file):
        conn = None
        try: 
            conn = sqlite3.connect(db_file)
            return conn
        except Error as e:
            print(e)
        return conn

    def create_table(self, conn, create_table_sql):
        try:
            c = conn.cursor()
            c.execute(create_table_sql)
        except Error as e: 
            print(e)


def main(args=None):
    rclpy.init(args=args)

    system_logger = Logger()
    rclpy.spin(system_logger)

    # Destroy the node explicitly
    system_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
