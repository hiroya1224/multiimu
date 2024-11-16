#!/usr/bin/env python3
import asyncio
import ssl
import os
from aiohttp import web
import sys
import mimetypes
import socket
import json
import numpy as np
import quaternion

import rospy
import roslib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

rospy.init_node('iphone_pose_pub', disable_signals=True)  # Disable rospy's signal handling

ROOT = os.path.join(roslib.packages.get_pkg_dir("multiimu"), "https-server")
HTTPS_ROOT = os.path.join(roslib.packages.get_pkg_dir("multiimu"), "controller")

CERTFILE = os.path.join(ROOT, "server.crt")
KEYFILE = os.path.join(ROOT, "server.key")

class ROSAsyncNode:
    def __init__(self, topic_name, topic_type=PoseStamped):
        # self.message_queue = asyncio.Queue()
        self.publisher = rospy.Publisher(topic_name, topic_type, queue_size=1)
        # rospy.Subscriber('/input_topic', PoseStamped, self.callback)

    # def callback(self, msg):
    #     asyncio.run_coroutine_threadsafe(self.message_queue.put(msg), asyncio.get_running_loop())

    # async def process_messages(self):
    #     while not rospy.is_shutdown():
    #         msg = await self.message_queue.get()
    #         rospy.loginfo(f"Processing message: {msg.data}")
    #         await asyncio.sleep(1)

    async def publish_messages(self, msg):
        # while not rospy.is_shutdown():
            # msg = String(data="Hello from asyncio")
            self.publisher.publish(msg)
            await asyncio.sleep(0.01)

# rospy.init_node("iphone_pose_pub")
# pub = rospy.Publisher("/iphone", PoseStamped)
        
ros_node1 = ROSAsyncNode("/iphone")
ros_node2 = ROSAsyncNode("/iphone_modified")
ros_node3 = ROSAsyncNode("/iphone_aligned")

ros_node_gripper = ROSAsyncNode("/gripper_control", String)

def get_local_ip():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        # GoogleのDNSサーバー（8.8.8.8）に接続してローカルIPアドレスを確認
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]

LOCALHOST = get_local_ip()

"""
openssl req -newkey rsa:2048 -nodes -keyout server.key -x509 -days 365 -out server.crt
"""

PORT = 4430

global server_process
server_process = None
# global ws_connections
global debug_viewer
# ws_connections = []  # WebSocket connection
debug_viewer = None

global shutdown_processing
shutdown_processing = False

# MIMEタイプを追加
mimetypes.add_type('application/javascript', '.js')
mimetypes.add_type('text/css', '.css')

class QuaternionHandler:
    def __init__(self):
        pass
    import math

    @staticmethod
    def matrix_to_quaternion(m):
        m00, m01, m02 = m[0]
        m10, m11, m12 = m[1]
        m20, m21, m22 = m[2]

        tr = m00 + m11 + m22

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2  # S=4*qw
            qw = 0.25 * S
            qx = (m21 - m12) / S
            qy = (m02 - m20) / S
            qz = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S=4*qx
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif m11 > m22:
            S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S=4*qy
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S=4*qz
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S

        return np.array([qw, qx, qy, qz])


    def align_xaxis(self, referenceQuaternion):
        ref = quaternion.as_quat_array(referenceQuaternion)
        refRot = quaternion.as_rotation_matrix(ref)
        
        x, y = refRot[:, 0], refRot[:, 1]

        x[2] = 0
        y[2] = 0

        x_nrml = x / np.linalg.norm(x)
        y_nrml = y / np.linalg.norm(y)
        z_nrml = np.cross(x_nrml, y_nrml)

        newRot = np.vstack([x_nrml, y_nrml, z_nrml]).T

        return newRot


qh = QuaternionHandler()


async def handle(request):
    path = request.match_info.get('path', '')

    if path == '':
        path = 'index.html'
    
    file_path = os.path.join(HTTPS_ROOT, path)

    if os.path.isdir(file_path):
        file_path = os.path.join(file_path, 'index.html')

    if os.path.exists(file_path) and os.path.isfile(file_path):
        ext = os.path.splitext(file_path)[1]
        content_type = mimetypes.guess_type(file_path)[0] or 'text/html'
        with open(file_path, "r") as f:
            content = f.read()
        return web.Response(text=content, content_type=content_type)

    return web.Response(status=404, text="404: Not Found")

reference_quat = quaternion.as_quat_array(np.array([1, 0, 0, 0]))

position_3D = np.array([0,0,0])

async def websocket_handler(request):
    global reference_quat, position_3D

    ws = web.WebSocketResponse()
    await ws.prepare(request)

    X = 0.
    Y = 0.
    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT:
            try:
                # JSONとしてメッセージをパースして表示
                data = json.loads(msg.data)
                control = data.get("control")
                action = data.get("action")

                ros_node_gripper.publish_messages(
                        msg.data
                )
                print("pub")

                if control == "joystick":
                    # X = action.get("x")
                    # Y = action.get("y")
                    X = -action.get("y")
                    Y = -action.get("x")
                    # print(f"Joystick position - x: {x}, y: {y}")

                # elif control == "deviceOrientation":
                #     # クォータニオンデータを取得
                #     w = action.get("w")
                #     x = action.get("x")
                #     y = action.get("y")
                #     z = action.get("z")
                #     # print(f"Device orientation (quaternion) - w: {w}, x: {x}, y: {y}, z: {z}")
                #     quat = np.array([w,x,y,z]) * np.sign(w)
                #     print(qh.align_xaxis(quat))
                #     quat_mod = qh.matrix_to_quaternion(qh.align_xaxis(quat))

                #     msg1 = PoseStamped()
                #     msg1.header.stamp = rospy.Time.now()
                #     msg1.header.frame_id = "world"
                #     msg1.pose.position.x = 0.
                #     msg1.pose.position.y = 0.
                #     msg1.pose.position.z = 0.
                #     msg1.pose.orientation.w = quat[0]
                #     msg1.pose.orientation.x = quat[1]
                #     msg1.pose.orientation.y = quat[2]
                #     msg1.pose.orientation.z = quat[3]

                #     await ros_node1.publish_messages(msg1)


                #     msg2 = PoseStamped()
                #     msg2.header.stamp = rospy.Time.now()
                #     msg2.header.frame_id = "world"
                #     msg2.pose.position.x = 1.
                #     msg2.pose.position.y = 0.
                #     msg2.pose.position.z = 0.
                #     msg2.pose.orientation.w = quat_mod[0]
                #     msg2.pose.orientation.x = quat_mod[1]
                #     msg2.pose.orientation.y = quat_mod[2]
                #     msg2.pose.orientation.z = quat_mod[3]


                #     await ros_node2.publish_messages(msg2)

                #     try:
                #         quat_aligned = quaternion.as_quat_array(quat) * reference_quat.conj()
                #         # print(quaternion.as_quat_array(quat))
                #         rot_aligned = quaternion.as_rotation_matrix(quat_aligned)

                #         print("rot_aligned @ np.array([X, Y, 0]))", rot_aligned @ np.array([X, Y, 0]))

                #         position_3D = position_3D + (rot_aligned @ np.array([X, Y, 0])) 

                #         msg3 = PoseStamped()
                #         msg3.header.stamp = rospy.Time.now()
                #         msg3.header.frame_id = "world"
                #         msg3.pose.position.x = position_3D[0]
                #         msg3.pose.position.y = position_3D[1]
                #         msg3.pose.position.z = position_3D[2]
                #         msg3.pose.orientation.w = quat_aligned.w
                #         msg3.pose.orientation.x = quat_aligned.x
                #         msg3.pose.orientation.y = quat_aligned.y
                #         msg3.pose.orientation.z = quat_aligned.z

                #         await ros_node3.publish_messages(msg3)
                #     except Exception as e:
                #         print(e)

                # elif control == "buttonY":
                #     if action == "press":
                #         reference_quat = quaternion.as_quat_array(quat_mod)

                # # elif control == "buttonX":
                # #     ros_node_gripper.publish_messages(
                # #         json.dumps({"control": "gripper_open_close", "button_action": action})
                # #     )
                # # elif control == "buttonY":
                # #     ros_node_gripper.publish_messages(
                # #         json.dumps({"control": "gripper_rotation", "button_action": action})
                # #     )
                    
                else:
                    print(f"Unknown control type: {control}")
                    pass

                print(data)

            except json.JSONDecodeError:
                print(f"Invalid JSON received: {msg.data}")

        elif msg.type == web.WSMsgType.ERROR:
            print(f'WebSocket connection closed with exception {ws.exception()}')

    print('WebSocket connection closed')
    return ws

async def init_http_app():
    app = web.Application()
    app.router.add_get('/{path:.*}', handle)
    return app

async def start_http_server():
    app = await init_http_app()

    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', PORT, ssl_context=ssl_context, reuse_address=True)
    await site.start()

    print(f"HTTP server running on https://{LOCALHOST}:{PORT}")

async def start_websocket_server():
    app = web.Application()
    app.router.add_get('/server_handler', websocket_handler)

    ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 9999, ssl_context=ssl_context, reuse_address=True)
    await site.start()

    print(f"WebSocket server running on wss://{LOCALHOST}:9999/server_handler")

async def main():
    await asyncio.gather(
        start_http_server(),
        start_websocket_server(),
        asyncio.Future()
    )

# Modify run_command to send stdout/stderr to WebSocket
async def run_command():
    global ws_connection  # Reference to the WebSocket connection
    command = [sys.executable, '-u', './ros/atoms3_ros_async.py', '--ros']

    if debug_viewer is not None:
        await debug_viewer.send_str("Waiting for process...")
    process = await asyncio.create_subprocess_exec(
        *command,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )
    if debug_viewer is not None:
        await debug_viewer.send_str("Process begining.")

    async for stdout_line in process.stdout:
        message = stdout_line.decode('utf-8').strip()
        print(message)
        if debug_viewer is not None:
            await debug_viewer.send_str(message)  # Send stdout to WebSocket

    async for stderr_line in process.stderr:
        error_message = f"Error: {stderr_line.decode('utf-8').strip()}"
        print(error_message)
        if debug_viewer is not None:
            await debug_viewer.send_str(error_message)  # Send stderr to WebSocket

    await process.wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
        if server_process is not None:
            server_process.cancel()