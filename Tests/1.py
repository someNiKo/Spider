# publisher.py
import zmq
import time
import string

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://127.0.0.1:5556")

# 等待订阅者连接（避免消息丢失）
time.sleep(0.2)

while True:
    for ch in string.ascii_lowercase:
        pub.send_string(ch)
        print("sent", ch)
        time.sleep(0.1)