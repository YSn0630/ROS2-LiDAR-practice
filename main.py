import roslibpy
import numpy as np
import time
import threading
import pymysql
import json

linear_x = 3.0
angular_z = 1.5

config = dict(
    host = 'localhost',
    user = 'root',
    password = '000630',
    database = 'scanmockdb',
    charset = 'utf8'
)

class DB:
    def __init__(self, **config):
        self.config = config

    def connect(self):
        return pymysql.connect(**self.config)

    def DBcreateTable(self):
        sql1 = "USE scanmockdb;"
        sql2 = "CREATE TABLE IF NOT EXISTS scanmocktb ( \
        id INT PRIMARY KEY AUTO_INCREMENT, \
        ranges json, \
        ts DATETIME, \
        act VARCHAR(50) \
        );"
        with self.connect() as con:
            with con.cursor() as cur:
                cur.execute(sql1)
                cur.execute(sql2)

    def insert_data(self, ranges, act):
        sql = "INSERT INTO scanmocktb (ranges, ts, act) VALUES (%s, now(), %s)"
        with self.connect() as con:
            try:
                with con.cursor() as cur:
                    cur.execute(sql, (ranges, act))
                con.commit()
                return True
            except Exception:
                con.rollback()
                return False


def compute_action(ranges):
    ranges = np.array(ranges)

    front = np.r_[ranges[350:360], ranges[0:10]]
    left  = ranges[80:100]
    right = ranges[260:280]

    front_dist = np.mean(front)
    left_dist  = np.mean(left)
    right_dist = np.mean(right)

    safe_dist = 0.5
    action = [0.0, 0.0]

    if front_dist < safe_dist:
        if left_dist > right_dist:
            am = "turn_left"
            action[1] = angular_z
        else:
            am = "turn_right"
            action[1] = -angular_z
    else:
        am = "go_forward"
        action[0] = linear_x

    return front_dist, left_dist, right_dist, action, am


class LidarClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.latest_scan = None

        print("Connecting to rosbridge...")
        self.client.run()
        print("Connected!\n")

        print("Connecting to database...")
        self.db = DB(**config)
        self.db.DBcreateTable()
        print("Connected!\n")

        self.scan_topic = roslibpy.Topic(
            self.client,
            '/scan_mock',
            'sensor_msgs/LaserScan'
        )
        self.scan_topic.subscribe(self.on_scan)

        self.cmd_topic = roslibpy.Topic(
            self.client,
            '/cmd_mock',
            'geometry_msgs/Twist'
        )

        self.thread = threading.Thread(target=self.loop)
        self.thread.daemon = True
        self.thread.start()

    def on_scan(self, msg):
        self.latest_scan = msg['ranges']

    def loop(self):
        while True:
            if self.latest_scan is not None:
                f, l, r, act, am = compute_action(self.latest_scan)
                self.cmd_topic.publish(roslibpy.Message({
                    'linear': {'x': act[0]},
                    'angular': {'z': act[1]}
                }))
                print("front:", round(f,2))
                print("left :", round(l,2))
                print("right:", round(r,2))
                print("action:", am)
                print("-------------------------------")
                self.db.insert_data(json.dumps(self.latest_scan), am)
            time.sleep(2)

    def close(self):
        self.scan_topic.unsubscribe()
        self.client.terminate()


if __name__ == "__main__":
    cli = LidarClient('localhost', 9090)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cli.close()
