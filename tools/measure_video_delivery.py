"""Measure actual MJPEG frame delivery through the public viewer, following redirects."""
import argparse,concurrent.futures,json,time,urllib.request
from pathlib import Path

def measure(base, name, seconds):
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    times=[];keys=[]
    with opener.open(base.rstrip('/')+'/'+name+'.mjpg',timeout=5) as response:
        resolved=response.geturl();start=time.monotonic()
        while time.monotonic()-start<seconds:
            line=response.readline()
            if not line:raise RuntimeError('Stream ended')
            if not line.startswith(b'--frame'):continue
            headers={}
            while True:
                line=response.readline().strip()
                if not line:break
                key,value=line.split(b':',1);headers[key.lower()]=value.strip()
            remaining=int(headers[b'content-length'])
            while remaining:
                chunk=response.read(remaining)
                if not chunk:raise RuntimeError('Truncated JPEG')
                remaining-=len(chunk)
            keys.append(headers.get(b'x-frame-key',b'').decode());times.append(time.monotonic())
    return dict(stream=name,resolved_url=resolved,frames=len(times),unique_keys=len(set(keys)),
                fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://192.168.50.20:8872')
    parser.add_argument('--seconds',type=float,default=20)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    with concurrent.futures.ThreadPoolExecutor() as pool:
        results=list(pool.map(lambda n:measure(args.url,n,args.seconds),['camera','infer']))
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(args.url+'/api/state',timeout=3) as r:state=json.load(r)
    data=dict(delivery=results,capture=state['pc'],inference_fps=state['nano']['data']['inference_fps'])
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(data,indent=2),encoding='utf8')
    print(json.dumps(data,indent=2))
if __name__=='__main__':main()
