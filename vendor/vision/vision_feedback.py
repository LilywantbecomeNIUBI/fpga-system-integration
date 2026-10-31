"""Query or inject one selected result from the configured slot-1 host; no YOLO dependency."""
import argparse
import json
import socket
from vision_protocol import PORT, detection, query, parse_status

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fpga', default='192.168.50.30')
    parser.add_argument('--local-ip', required=True, help='must be the configured slot-1 receiver address')
    parser.add_argument('--frame-id', type=int, help='omit to query status without changing the target')
    parser.add_argument('--bbox', type=int, nargs=4, metavar=('X1', 'Y1', 'X2', 'Y2'))
    parser.add_argument('--class-id', type=int, default=39)
    parser.add_argument('--confidence', type=int, default=900)
    parser.add_argument('--inference-ms', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=2)
    args = parser.parse_args()
    if args.frame_id is None and args.bbox is not None:
        parser.error('--bbox requires --frame-id')
    payload = query() if args.frame_id is None else detection(
        args.frame_id, args.bbox, args.class_id, args.confidence, inference_ms=args.inference_ms)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.local_ip, 0))
        sock.settimeout(args.timeout)
        sock.connect((args.fpga, PORT))
        sock.send(payload)
        print(json.dumps(parse_status(sock.recv(2048)), indent=2))

if __name__ == '__main__':
    main()
