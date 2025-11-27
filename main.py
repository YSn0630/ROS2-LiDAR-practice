import roslibpy
import numpy as np
import time
import threading

### 분석 영역 설정 ###
def compute_action(ranges):
    ranges = np.array(ranges)

    # LDS-02 기준 angle_increment = 1도
    front = np.r_[ranges[350:360], ranges[0:10]]
    left  = ranges[80:100]
    right = ranges[260:280]

    front_dist = np.mean(front)
    left_dist  = np.mean(left)
    right_dist = np.mean(right)

    safe_dist = 0.5

    if front_dist < safe_dist:
        action = "turn_left" if left_dist > right_dist else "turn_right"
    else:
        action = "go_forward"

    return front_dist, left_dist, right_dist, action


### rosbridge 연결 + /ls_mock 구독 ###
class LidarClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.latest_scan = None

        print("Connecting to rosbridge...")
        self.client.run()
        print("Connected!")

        self.scan_topic = roslibpy.Topic(
            self.client,
            '/ls_mock',
            'pubscan_pkg_msgs/msg/LaserScanmock'
        )
        self.scan_topic.subscribe(self.on_lidar)

        # 2초마다 액션 출력하는 스레드
        self.print_thread = threading.Thread(target=self.print_loop)
        self.print_thread.daemon = True
        self.print_thread.start()

    def on_lidar(self, message):
        """ /ls_mock 메시지를 받을 때마다 실행 """
        self.latest_scan = message["ranges"]

    def print_loop(self):
        while True:
            if self.latest_scan is not None:
                f, l, r, act = compute_action(self.latest_scan)

                print("front:", round(f, 2))
                print("left :", round(l, 2))
                print("right:", round(r, 2))
                print("action:", act)
                print("-------------------------------")
            time.sleep(2.0)

    def close(self):
        self.scan_topic.unsubscribe()
        self.client.terminate()


### 실행 ###
if __name__ == "__main__":
    # Ubuntu ROS2 머신의 IP 입력
    lidar = LidarClient(host='localhost', port=9090)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        lidar.close()
        print("Closed client.")
