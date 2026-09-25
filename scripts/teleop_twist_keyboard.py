#!/usr/bin/env python3

# Brown University Robotics による 2011 年の著作権表記。
# Open Source Robotics Foundation, Inc. による 2017 年の著作権表記。
# 無断複写・転載を禁じます。
#
# BSD ライセンス 2.0 に基づくソフトウェア使用許諾契約です。
#
# ソース形式およびバイナリ形式での再配布と利用は、
# 変更の有無にかかわらず、以下の条件を満たす場合に許可されます。
#
#  * ソースコードの再配布では、上記の著作権表示、この条件一覧、
#    および以下の免責事項を保持しなければなりません。
#  * バイナリ形式の再配布では、配布物に付属する文書やその他の資料の中に、
#    上記の著作権表示、この条件一覧、
#    および以下の免責事項を再掲しなければなりません。
#  * Willow Garage の名称、またはその協力者の名称を、
#    本ソフトウェアに基づく製品の推奨や宣伝に使用してはなりません。
#
# 本ソフトウェアは、著作権者および協力者によって「現状のまま」で提供されます。
# 商品性および特定目的への適合性に関する黙示の保証を含め、
# 明示または黙示のいかなる保証も否認されます。
# いかなる場合も、著作権者または協力者は、直接的、間接的、偶発的、特別、
# 典型的、または結果的損害について責任を負いません。
# これには、代替品またはサービスの調達、使用不能、データ損失、逸失利益、
# 事業中断が含まれますが、これらに限定されません。
# その責任は、契約、不法行為、厳格責任、または過失を含むいかなる理論によっても、
# 本ソフトウェアの使用に起因して生じるものです。
# たとえそのような損害の可能性が事前に通知されていたとしても、同様です。

import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
    import select


msg = """
このノードはキーボード入力を受け取り、Twist/TwistStamped メッセージとして publish します。
US 配列のキーボードでの使用が最適です。
---------------------------
移動:
    u    i    o
    j    k    l
    m    ,    .

回転は、Shift キーを押しながら使います:
---------------------------
    U    I    O
    J    K    L
    M    <    >

t : 上方向 (+z)
b : 下方向 (-z)

それ以外 : 停止

q/z : 最大速度を 10% 増減
w/x : 直進速度のみを 10% 増減
e/c : 角速度のみを 10% 増減

CTRL-C で終了
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, -1, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (1, 1, 0, 0),
    ',': (-1, 0, 0, 0),
    '.': (-1, -1, 0, 0),
    'm': (-1, 1, 0, 0),
    'O': (1, 0, 0, -1),
    'I': (1, 0, 0, 0),
    'J': (0, 0, 0, 1),
    'L': (0, 0, 0, -1),
    'U': (1, 0, 0, 1),
    '<': (-1, 0, 0, 0),
    '>': (-1, 0, 0, 1),
    'M': (-1, 0, 0, -1),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey(settings, timeout=0.02):
    """
    ノンブロッキングで1文字読み取る。タイムアウト内に入力がなければ空文字を返す。
    """
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            return msvcrt.getwch()
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
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

    node = rclpy.create_node('teleop_twist_keyboard')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only_descriptor).value
    frame_id = node.declare_parameter('frame_id', '', read_only_descriptor).value
    speed = node.declare_parameter('speed', 0.5, read_only_descriptor).value
    turn = node.declare_parameter('turn', 1.0, read_only_descriptor).value

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                y = moveBindings[key][0]
                x = -1 * moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == '':
                # 入力なし: 直前の指示値を維持してそのまま publish する
                pass
            else:
                # 未定義のキー: 停止コマンド
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == '\x03':
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()