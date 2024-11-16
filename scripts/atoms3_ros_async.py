#!/usr/bin/env python3
import socket
import time
import ssl
import asyncio
import websockets
import numpy as np
import json
import os
from pathlib import Path
from imusocket import *

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from urdf_estimation_with_imus.msg import PoseWithCovAndBingham

import roslib
ROOT = os.path.join(roslib.packages.get_pkg_dir("multiimu"), "https-server")
CERTFILE = os.path.join(ROOT, "server.crt")
KEYFILE = os.path.join(ROOT, "server.key")
# ROOT = "/home/leus/sensor-https-server/"

class SSLContextManager:
    @staticmethod
    def create_context(certfile, keyfile=None):
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        if keyfile:
            context.load_cert_chain(certfile=certfile, keyfile=keyfile)
        else:
            context.load_cert_chain(certfile=certfile)
        return context

context = SSLContextManager.create_context(CERTFILE, KEYFILE)
# context2 = SSLContextManager.create_context(ROOT + "localhost.pem")

# global RESETESTIM_PUB
# RESETESTIM_PUB = None


rospy.init_node("websocket_node", anonymous=True, disable_signals=True)
rospy.loginfo("Initialized node.")

RESETESTIM_PUB = rospy.Publisher("/reset_estimation", Empty, queue_size=1)

# class RelativeRotationWebsocketNode:
#     def __init__(self):
#         rospy.Subscriber("/estimated_relative_pose/imucdfe20__to__imua01730", PoseWithCovAndBingham, self.callback)
#         self.Avec = self.Amat_to_AvecList(np.eye(4) * 1e-6)

#     @staticmethod
#     def Amat_to_AvecList(Amat):
#         return Amat[np.triu_indices(4)].tolist()

#     def callback(self, msg):
#         self.Avec = self.Amat_to_AvecList(np.array(msg.rotation_bingham_parameter).reshape(4, 4))

#     async def handler(self, websocket):
#         async for msg in websocket:
#             dataset_dict = {"Avec": self.Avec}
#             await websocket.send(json.dumps(dataset_dict))
#             await asyncio.sleep(0.1)


class TopicHolder:
    def __init__(self, ip_and_mac):
        self.datalist_for_rospub = []
        self.datalist_for_visualize = []
        self.client = ImuDataSocketClient(host=ip_and_mac[0])
        self.ip = ip_and_mac[0]
        self.mac = ip_and_mac[1]

    def update_dataset(self, datalist):
        for dl in [self.datalist_for_rospub, self.datalist_for_visualize]:
            dl.extend(datalist)

    def get_dataset_as_rosmsg_list(self):
        if not self.datalist_for_rospub:
            return []
        msgs = [self.create_imu_msg(d) for d in self.datalist_for_rospub]
        self.datalist_for_rospub = []
        return msgs

    def create_imu_msg(self, data):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(data["timestamp"])
        msg.header.frame_id = f"imu{data['data']['mac']}__{data['data']['mac']}"
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = data["data"]["gyr"]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = data["data"]["acc"]
        return msg


class TopicHolderList:
    def __init__(self):
        self.tclist = []
        self.registered_host = []
        self.registered_info = []
        self.is_registered_new_imu = True
        self.imus_to_be_visualized = ["", ""]

    def __iter__(self):
        return iter(self.tclist)

    def get_tc_from_macaddress(self, mac):
        return next((i for i, tc in enumerate(self.tclist) if tc.mac == mac), -1)

    def get_dataset_as_json_for_visualize(self, only_lastelem=True):
        try:
            dataset1, dataset2 = self.get_visual_datasets()
            if only_lastelem:
                json_str = json.dumps({"message_type": "data", "body": [dataset1[-1], dataset2[-1]]})
            else:
                json_str = json.dumps({"message_type": "data", "body": [dataset1, dataset2]})
            self.clear_visual_datasets()
            return json_str
        except Exception as e:
            print(e)

    def get_visual_datasets(self):
        idx1 = self.get_tc_from_macaddress(self.imus_to_be_visualized[0])
        idx2 = self.get_tc_from_macaddress(self.imus_to_be_visualized[1])
        dataset1 = self.tclist[idx1].datalist_for_visualize if idx1 >= 0 else []
        dataset2 = self.tclist[idx2].datalist_for_visualize if idx2 >= 0 else []
        return dataset1, dataset2

    def clear_visual_datasets(self):
        for tc in self.tclist:
            tc.datalist_for_visualize = []

    def format_registered_info_for_html(self, regiinfo):
        return f"[{regiinfo['time']:.6f}] Registered IMU {regiinfo['mac']} ({regiinfo['ip']})."

    def register_imu_info(self, registered_time, ip_address, mac_address):
        info = {"time": registered_time, "ip": ip_address, "mac": mac_address}
        self.registered_info.append(info)
        return info


tcs = TopicHolderList()


def check_ip_address(msg):
    try:
        ip, mac = msg.split(":")
        ip_nums = [int(n) for n in ip.split(".")]
        return all(0 <= n < 256 for n in ip_nums) and len(mac) == 6, (ip, mac)
    except:
        return False, None


async def fetch_dataset():
    while True:
        for tc in tcs:
            try:
                tc.update_dataset(tc.client.received_data_as_dict())
            except UnicodeDecodeError:
                continue
        await asyncio.sleep(0.001)


async def ros_publish():
    pub = rospy.Publisher("/imu", Imu, queue_size=10)
    while True:
        for tc in tcs:
            for msg in tc.get_dataset_as_rosmsg_list():
                pub.publish(msg)
        await asyncio.sleep(0.001)


class WebSocketServer:
    def __init__(self, context, port, handler):
        self.context = context
        self.port = port
        self.handler = handler

    async def run(self):
        async with websockets.serve(self.handler, "0.0.0.0", self.port, ssl=self.context):
            await asyncio.Future()


async def visualize_handler(websocket):
    print("Connected to Visualizer")
    tcs.is_registered_new_imu = True
    tcs.imus_to_be_visualized = ["", ""]
    async for message in websocket:
        if tcs.is_registered_new_imu:
            imu_info = json.dumps({"message_type": "imu_info", "registered_imus": tcs.registered_info})
            tcs.is_registered_new_imu = False
            await websocket.send(imu_info)

        msg = json.loads(message)
        request = msg["request"]
        try:
            if request == "go":
                await websocket.send(tcs.get_dataset_as_json_for_visualize(only_lastelem=False))
            elif request == "imu_name":
                tcs.imus_to_be_visualized = msg["name_list"]
                await websocket.send(json.dumps({"message_type": "continue"}))
        except Exception as e:
            print(e)
        await asyncio.sleep(0.01)


async def qrcode_handler(websocket):
    print("Connected to QR Reader")
    for ri in tcs.registered_info:
        await websocket.send(tcs.format_registered_info_for_html(ri))

    async for _message in websocket:
        print(f"received: {_message}")
        _message_json = json.loads(_message)

        if _message_json["request"] == "decodedResult":

            message = _message_json["text"]
            if message == "reset":
                tcs.registered_host = []
                await websocket.send("reset")

            is_valid, ip_and_mac = check_ip_address(message)
            if is_valid and message not in tcs.registered_host:
                tcs.registered_host.append(message)
                tc = TopicHolder(ip_and_mac)
                tcs.tclist.append(tc)
                regiinfo_dict = tcs.register_imu_info(time.time(), ip_and_mac[0], ip_and_mac[1])
                tc.client.send_registered_signal()
                tcs.is_registered_new_imu = True
                await websocket.send(tcs.format_registered_info_for_html(regiinfo_dict))

        elif _message_json["request"] == "resetEstimation":
            RESETESTIM_PUB.publish()
            


async def main():
    tasks = [
        asyncio.create_task(fetch_dataset()),
        asyncio.create_task(ros_publish()),
        asyncio.create_task(WebSocketServer(context, 8001, visualize_handler).run()),
        asyncio.create_task(WebSocketServer(context, 8003, qrcode_handler).run()),
        # asyncio.create_task(WebSocketServer(context, 8002, RelativeRotationWebsocketNode().handler).run())
    ]
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--host', help="IP address of the sensor module")
    # parser.add_argument('--ros', help="Connecting to ROS", action="store_false")
    # args = parser.parse_args()
    if True:
        # rospy.init_node("websocket_node", anonymous=True, disable_signals=True)
        # rospy.loginfo("Initialized node.")

        # RESETESTIM_PUB = rospy.Publisher("/reset_estimation", Empty, queue_size=1)
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            rospy.loginfo("Finished node.")
            rospy.signal_shutdown('finish')
    else:
        asyncio.run(main())
