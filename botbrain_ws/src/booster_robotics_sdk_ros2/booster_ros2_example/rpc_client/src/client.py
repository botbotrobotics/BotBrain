import rclpy
from rclpy.node import Node
from booster_interface.srv import RpcService
from booster_interface.msg import BoosterApiReqMsg
import json
import time

def create_msg(api_id, param_dict=None):
    msg = BoosterApiReqMsg()
    msg.api_id = api_id
    if param_dict is not None:
        msg.body = json.dumps(param_dict)
    else:
        msg.body = ""
    return msg

def main():
    rclpy.init()
    node = Node('rpc_client_node')
    client = node.create_client(RpcService, 'booster_rpc_service')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
        if not rclpy.ok():
            node.get_logger().error('Interrupted while waiting for the service. Exiting.')
            return

    # 构造消息
    # 切换到手末端控制模式
    # 2012 is the API ID for kSwitchHandEndEffectorControlMode,refer to the b1_loco_api.hpp
    req_msg = create_msg(2012, {"switch_on": True})  # kSwitchHandEndEffectorControlMode
    # 关闭手末端控制模式
    req_msg_close = create_msg(2012, {"switch_on": False})
    # 移动
    req_move_msg = create_msg(2001, {"vx": 0.5, "vy": 0.0, "vyaw": 0.0})  # kMove
    # 停止
    req_move_msg_zero = create_msg(2001, {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})
    # 移动手末端
    tar_posture = {
        "position_": {"x": 0.35, "y": 0.25, "z": 0.1},
        "orientation_": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    }
    req_move_hand_end_effector_msg = create_msg(
        2009,  # kMoveHandEndEffector
        {
            "target_posture": tar_posture,
            "time_millis": 2000,
            "hand_index": 0  # kLeftHand
        }
    )

    while rclpy.ok():
        # 切换到手末端控制模式
        request = RpcService.Request()
        request.msg = req_msg
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info('Result: %s' % future.result().msg.body)
        else:
            node.get_logger().error('Failed to call rpc service')
        time.sleep(2)

        # 移动
        request.msg = req_move_msg
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info('Result: %s' % future.result().msg.body)
        else:
            node.get_logger().error('Failed to call rpc service')
        time.sleep(2)

        # 移动手末端
        request.msg = req_move_hand_end_effector_msg
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info('Result: %s' % future.result().msg.body)
        else:
            node.get_logger().error('Failed to call rpc service')
        time.sleep(20)

        # 停止
        request.msg = req_move_msg_zero
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info('Result: %s' % future.result().msg.body)
        else:
            node.get_logger().error('Failed to call rpc service')
        time.sleep(2)

        # 关闭手末端控制模式
        request.msg = req_msg_close
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info('Result: %s' % future.result().msg.body)
        else:
            node.get_logger().error('Failed to call rpc service')
        time.sleep(10)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()