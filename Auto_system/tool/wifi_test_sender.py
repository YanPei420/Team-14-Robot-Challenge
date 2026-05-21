import argparse
import signal
import sys
import time

import paho.mqtt.client as mqtt


def build_topic(group_id: str, from_board: str, target: str) -> str:
    return f"lab/g/{group_id}/from/{from_board}/to/{target}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send MQTT heartbeat/test messages to test/wifi.cpp."
    )
    parser.add_argument("--host", default="192.168.0.74")
    parser.add_argument("--port", default=1883, type=int)
    parser.add_argument("--group", default="14")
    parser.add_argument("--robot", default="Robot14")
    parser.add_argument("--server", default="server")
    parser.add_argument("--interval", default=0.5, type=float)
    parser.add_argument(
        "--payload",
        default="type=heartbeat enable=1",
        help="Payload to send repeatedly.",
    )
    args = parser.parse_args()

    topic = build_topic(args.group, args.server, args.robot)
    stop_payload = "type=stop"
    running = True

    def request_stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    client = mqtt.Client(client_id=f"{args.server}-python-test")
    client.connect(args.host, args.port, keepalive=30)
    client.loop_start()

    print(f"MQTT broker: {args.host}:{args.port}")
    print(f"Publishing to: {topic}")
    print(f"Repeated payload: {args.payload!r}")
    print("Press Ctrl+C to send stop and exit.")

    try:
        while running:
            client.publish(topic, args.payload, qos=0, retain=False)
            time.sleep(args.interval)
    finally:
        client.publish(topic, stop_payload, qos=0, retain=False)
        time.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("Sent stop.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
