import math
import time

import rclpy
from geometry_msgs.msg import Twist

# 過去の見本データ
# [門までの距離, 門に近づく速度, 危険かどうか]
training_data = [
    (5.0, 0.1, 0),
    (4.0, 0.3, 0),
    (3.0, 0.2, 0),
    (2.5, 0.1, 0),
    (2.0, 0.1, 0),
    (1.5, 0.0, 0),
    (1.6, 0.4, 1),
    (1.2, 0.3, 1),
    (1.0, 0.5, 1),
    (0.6, 0.2, 1),
    (0.3, 0.4, 1),
]

# AI：k近傍法（k=3）
def predict_risk(distance, speed):
    neighbors = sorted(
        training_data,
        key=lambda row: math.hypot(
            row[0] - distance,
            3 * (row[1] - speed)
        )
    )
    votes = sum(row[2] for row in neighbors[:3])
    return votes >= 2


def main():
    rclpy.init()
    node = rclpy.create_node("mimamori_ai")
    publisher = node.create_publisher(
        Twist, "/turtle1/cmd_vel", 10
    )

    print("=== みまもりAI 起動 ===", flush=True)
    print("園庭の巡回を開始します", flush=True)

    start = time.monotonic()
    last_print = -1

    try:
        while time.monotonic() - start < 75:
            elapsed = time.monotonic() - start

            # 架空の園児が門に近づく
            distance = max(0.2, 5.0 - elapsed * 0.065)
            speed = 0.4

            # AIが危険度を判断
            danger = predict_risk(distance, speed)

            command = Twist()

            if danger:
                command.linear.x = 0.0
                command.angular.z = 0.0
                status = "危険！巡回停止・職員に警告"
            else:
                command.linear.x = 0.6
                command.angular.z = 0.7
                status = "正常・巡回中"

            publisher.publish(command)

            second = int(elapsed)
            if second % 5 == 0 and second != last_print:
                print(
                    f"{second:02d}秒 | 門まで {distance:.1f}m "
                    f"| AI判定: {status}",
                    flush=True
                )
                last_print = second

            time.sleep(0.1)

    finally:
        publisher.publish(Twist())
        print("=== 実行終了 ===", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
