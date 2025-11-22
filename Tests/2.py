# subscriber.py
import zmq

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

print("subscriber started, waiting messages...")
while True:
    msg = sub.recv_string()
    print("recv", msg)