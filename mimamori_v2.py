"""みまもりAI v2: 園庭の飛び出しリスク判定ロボット（ROS 2 / Turtlesim）

登場する2匹の亀:
  turtle1 = 園児役（40秒後に門へ向かう模擬シナリオ）
  robot   = 見守りロボ（AIが危険と判定すると追いかける）
  赤い線  = 門（画面右側）

亀の実際の座標（/turtle1/pose）を取得し、門までの距離と接近速度を計算。
手作業で作った見本データ19件に対し、k近傍法（k=3）で危険/正常を推論する。
実際の園児の検知、画像認識、通知機能は含まない。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen, Spawn, TeleportAbsolute

# Turtlesimの座標は0～11程度。便宜上、距離をmと表記する。
GATE_X = 10.3
GATE_Y_MIN, GATE_Y_MAX = 3.0, 8.0
WANDER_SECONDS = 40
CHILD_RUN_SPEED = 0.5
CHILD_START = (2.0, 3.0, 0.0)
ROBOT_START = (3.0, 9.0, 0.0)
ROBOT_PATROL = (0.9, -0.5)
ROBOT_CHASE_SPEED = 1.5
CATCH_DISTANCE = 1.0
MAX_SECONDS = 120

# 見本データ: (門までの距離, 門への接近速度, 危険なら1/正常なら0)
# 19件とも検証用に手作業で設定した仮想データ。
training_data = [
    (8.0, 0.0, 0), (8.0, 0.8, 0), (7.0, 0.4, 0), (6.0, 0.5, 0), (6.0, 0.0, 0),
    (5.0, 0.3, 0), (4.0, 0.0, 0), (3.5, 0.1, 0), (3.0, 0.0, 0), (2.0, 0.0, 0),
    (4.5, 0.6, 1), (4.0, 0.5, 1), (3.0, 0.4, 1), (3.0, 0.6, 1), (2.0, 0.3, 1),
    (2.0, 0.5, 1), (1.5, 0.2, 1), (1.0, 0.5, 1), (1.0, 0.3, 1),
]
SPEED_WEIGHT = 4


def predict_risk(distance, approach_speed):
    """近い見本3件の多数決で危険ならTrue。"""
    neighbors = sorted(
        training_data,
        key=lambda row: math.hypot(
            row[0] - distance,
            SPEED_WEIGHT * (row[1] - approach_speed),
        ),
    )
    return sum(row[2] for row in neighbors[:3]) >= 2


def gate_distance(x, y):
    """門の赤い線分までの最短距離。"""
    dy = max(0.0, GATE_Y_MIN - y, y - GATE_Y_MAX)
    return math.hypot(GATE_X - x, dy)


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def go_to(pose, tx, ty, max_speed):
    """目的地に向けて旋回し、向きが合えば前進する。"""
    err = angle_diff(math.atan2(ty - pose.y, tx - pose.x), pose.theta)
    cmd = Twist()
    cmd.angular.z = 3.0 * err
    cmd.linear.x = max_speed * max(0.0, math.cos(err))
    return cmd


def call(node, client, request):
    """ROS 2サービスに依頼し、結果を待つ。"""
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"ROS 2サービスに接続できません: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"ROS 2サービスでエラー: {client.srv_name}")
    return future.result()


def spawn(node, name, x, y, theta):
    req = Spawn.Request()
    req.x, req.y, req.theta, req.name = float(x), float(y), float(theta), name
    call(node, node.create_client(Spawn, "/spawn"), req)


def kill(node, name):
    req = Kill.Request()
    req.name = name
    call(node, node.create_client(Kill, "/kill"), req)


def set_pen(node, name, r, g, b, width, off):
    req = SetPen.Request()
    req.r, req.g, req.b, req.width, req.off = r, g, b, width, off
    call(node, node.create_client(SetPen, f"/{name}/set_pen"), req)


def teleport(node, name, x, y, theta):
    req = TeleportAbsolute.Request()
    req.x, req.y, req.theta = float(x), float(y), float(theta)
    call(node, node.create_client(TeleportAbsolute, f"/{name}/teleport_absolute"), req)


def draw_gate(node):
    """門の赤い線を描き、線を引いた亀は消す。"""
    spawn(node, "gate", GATE_X, GATE_Y_MIN, math.pi / 2)
    set_pen(node, "gate", 255, 60, 60, 10, 0)
    publisher = node.create_publisher(Twist, "/gate/cmd_vel", 10)
    time.sleep(1.0)
    cmd = Twist()
    cmd.linear.x = 2.5
    end = time.monotonic() + (GATE_Y_MAX - GATE_Y_MIN) / 2.5
    while time.monotonic() < end:
        publisher.publish(cmd)
        time.sleep(0.1)
    publisher.publish(Twist())
    time.sleep(0.3)
    kill(node, "gate")


def main():
    rclpy.init()
    node = rclpy.create_node("mimamori_ai_v2")
    poses = {}
    node.create_subscription(
        Pose, "/turtle1/pose", lambda msg: poses.__setitem__("child", msg), 10
    )
    child_pub = node.create_publisher(Twist, "/turtle1/cmd_vel", 10)
    robot_pub = None

    try:
        # 園庭の舞台を準備する。
        set_pen(node, "turtle1", 0, 0, 0, 3, 1)
        teleport(node, "turtle1", *CHILD_START)
        set_pen(node, "turtle1", 179, 184, 255, 3, 0)
        spawn(node, "robot", *ROBOT_START)
        set_pen(node, "robot", 255, 200, 0, 2, 0)
        node.create_subscription(
            Pose, "/robot/pose", lambda msg: poses.__setitem__("robot", msg), 10
        )
        robot_pub = node.create_publisher(Twist, "/robot/cmd_vel", 10)
        draw_gate(node)

        while "child" not in poses or "robot" not in poses:
            rclpy.spin_once(node, timeout_sec=0.1)

        print("=== みまもりAI v2 起動 ===", flush=True)
        print("園児役: 40秒間遊び、その後、赤い門に向かいます", flush=True)
        print("見守りロボ: 危険判定後に追跡します", flush=True)

        start = prev_t = time.monotonic()
        prev_dist = None
        approach = 0.0
        alarm = False
        caught_at = None
        last_log = -1
        run_started = False
        result = "時間切れ"

        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            elapsed = now - start
            dt = max(now - prev_t, 1e-3)
            prev_t = now
            if elapsed > MAX_SECONDS:
                break

            child, robot = poses["child"], poses["robot"]
            dist = gate_distance(child.x, child.y)
            if prev_dist is not None:
                approach = 0.7 * approach + 0.3 * ((prev_dist - dist) / dt)
            prev_dist = dist
            speed = max(0.0, approach)

            if caught_at is None and predict_risk(dist, speed) and not alarm:
                alarm = True
                print(
                    f"{int(elapsed):02d}秒 | 【危険】AIが飛び出しリスクを検知。ロボが急行！"
                    f"（距離={dist:.1f}m / 接近速度={speed:.2f}m/秒）",
                    flush=True,
                )
                print("※職員への実際の通知は未実装です", flush=True)

            child_cmd = Twist()
            if caught_at is not None:
                pass
            elif elapsed < WANDER_SECONDS:
                child_cmd.linear.x, child_cmd.angular.z = 0.8, 0.8
            elif dist < 0.5:
                result = "園児役が門に到達（失敗）"
                print(f"{int(elapsed):02d}秒 | 【失敗】園児役が門に到達", flush=True)
                break
            else:
                if not run_started:
                    run_started = True
                    print(f"{int(elapsed):02d}秒 | 園児役が門に向かい始めました", flush=True)
                child_cmd = go_to(child, GATE_X - 0.3, 5.5, CHILD_RUN_SPEED)

            if caught_at is not None:
                robot_cmd = Twist()
            elif alarm:
                robot_cmd = go_to(robot, child.x, child.y, ROBOT_CHASE_SPEED)
                if math.hypot(robot.x - child.x, robot.y - child.y) < CATCH_DISTANCE:
                    caught_at = elapsed
                    result = "見守りロボが園児役に追いつき、停止"
                    print(
                        f"{int(elapsed):02d}秒 | 【成功】園児役の近くに到着。両方停止。"
                        f"（門まで {dist:.1f}m）",
                        flush=True,
                    )
                    robot_cmd = Twist()
                    child_cmd = Twist()
            else:
                robot_cmd = Twist()
                robot_cmd.linear.x, robot_cmd.angular.z = ROBOT_PATROL

            child_pub.publish(child_cmd)
            robot_pub.publish(robot_cmd)

            second = int(elapsed)
            if second % 2 == 0 and second != last_log and caught_at is None:
                label = "危険・追跡中" if alarm else "正常・巡回中"
                print(
                    f"{second:02d}秒 | 門まで {dist:.1f}m | 接近速度 {speed:.2f}m/秒 "
                    f"| AI判定: {label}",
                    flush=True,
                )
                last_log = second

            if caught_at is not None and elapsed - caught_at > 6:
                break
            time.sleep(0.1)

        print(f"=== 実行終了：{result} ===", flush=True)
    finally:
        child_pub.publish(Twist())
        if robot_pub is not None:
            robot_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
