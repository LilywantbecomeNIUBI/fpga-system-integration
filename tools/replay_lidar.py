"""Read an existing STL-27L capture; no serial device access."""
from pathlib import Path
import argparse,hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shared.streams import LidarStream
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--input',type=Path,required=True);a=ap.parse_args()
    raw=a.input.read_bytes();decoder=LidarStream()
    for i in range(0,len(raw),4096):decoder.feed(raw[i:i+4096])
    report=dict(input=str(a.input),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw),frames=decoder.frames,complete_scans=decoder.scans,crc_errors=decoder.bad_crc,hardware_tested_now=False,distance_accuracy_verified=False)
    out=ROOT/'build/tests';out.mkdir(parents=True,exist_ok=True);(out/'lidar_replay.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2))
    if decoder.frames==0 or decoder.bad_crc:raise SystemExit(1)
if __name__=='__main__':main()
