#!/usr/bin/env python3

import sys
import threading

import rcl_interfaces.msg
import rclpy
from uec_msgs.msg import CANArray, CAN

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# Publish destination
CAN_PORT = 2
CAN_TX_TOPIC = f"/can/tx/port{CAN_PORT}"

msg = """
このノードはキーボード入力を受け取り、各モータの指令値を CANArray メッセージとして
/can/tx に publish します。

US配列キーボードでの使用を想定しています。

--------------------------------
モータ番号とキーの対応（1〜10番）：

増加（+1.0 ずつ加算）:
    q  w  e  r  t  y  u  i  o  p
    |  |  |  |  |  |  |  |  |  |
    1  2  3  4  5  6  7  8  9 10

減少（-1.0 ずつ減算）:
    z  x  c  v  b  n  m  ,  .  /

停止（そのモータの値を 0.0 にリセット）:
    a  s  d  f  g  h  j  k  l  ;

--------------------------------
このノードの特徴：

・キーを押すたびに、そのモータの指令値が増減します（値は保持されます）
・現在の値がそのままCANメッセージとして送信されます
・モータごとの出力を手動で細かく調整するデバッグ用途に適しています

例：
    q を3回押す → モータ1 の値は 3.0
    z を1回押す → モータ1 の値は 2.0
    a を押す   → モータ1 の値は 0.0

--------------------------------
CTRL-C で終了します。
終了時には全モータに 0.0（停止）が送信されます。
"""

# Key sets mapped to motor indices (0-based)
# increase_keys[i] increases motor (i+1)
increase_keys = ['q','w','e','r','t','y','u','i','o','p']
# decrease keys map to motors 1..10
decrease_keys = ['z','x','c','v','b','n','m',',','.','/']
# stop keys map to motors 1..10
stop_keys = ['a','s','d','f','g','h','j','k','l',';']


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_can_array_keyboard')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value

    pub = node.create_publisher(CANArray, CAN_TX_TOPIC, 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        # per-motor speeds (index 0 -> motor 1)
        motor_speeds = [0.0] * 10

        while True:
            key = getKey(settings)
            if key == '\x03':   # Ctrl+C
                break

            # increase
            if key in increase_keys:
                idx = increase_keys.index(key)
                motor_speeds[idx] += 1.0
                val = motor_speeds[idx]
                motor_num = idx + 1

            # decrease
            elif key in decrease_keys:
                idx = decrease_keys.index(key)
                motor_speeds[idx] -= 1.0
                val = motor_speeds[idx]
                motor_num = idx + 1

            # stop
            elif key in stop_keys:
                idx = stop_keys.index(key)
                motor_speeds[idx] = 0.0
                val = 0.0
                motor_num = idx + 1

            else:
                # 未割り当てキーは何もしない
                continue

            can_msg = CAN()
            can_msg.id = 0
            can_msg.bulk_id = motor_num
            can_msg.data = [float(val)]

            can_array_msg = CANArray()
            can_array_msg.array = [can_msg]
            pub.publish(can_array_msg)
            status = (status + 1) % 15

    except Exception as e:
        print(e)

    finally:
        # 停止メッセージを送信
        can_array_msg = CANArray()
        # send stop (0.0) for motors 1..10
        can_list = []
        for m in range(1, 11):
            stop_msg = CAN()
            stop_msg.id = 0
            stop_msg.bulk_id = m
            stop_msg.data = [0.0]
            can_list.append(stop_msg)
        can_array_msg.array = can_list
        pub.publish(can_array_msg)
        
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()