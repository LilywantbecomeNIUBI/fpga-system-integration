"""Analyze a recorded trial's AB trace; no serial, HTTP or motor commands."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream
from shared.encoder_trace import TraceCollector,replay

def analyze(trial,axis):
    data=json.loads(trial.read_text());key=str(axis)
    samples=[r['encoder_traces'][key] for r in data['rows'] if key in r.get('encoder_traces',{})]
    if len(samples)<2:raise ValueError('Insufficient trace snapshots; trial requires trace firmware')
    first,last=samples[0]['event_count'],samples[-1]['event_count']
    if last<first:raise ValueError('Counter restart/wrap inside trial; split the recording')
    collector=TraceCollector(axis);parser=MixedStream();events=[]
    for raw in sorted((trial.parent/'gateway').glob('*/fpga_uart.bin')):
        with raw.open('rb') as stream:
            while True:
                chunk=stream.read(65536)
                if not chunk:break
                for kind,row in parser.feed(chunk):
                    if kind=='encoder_trace':
                        events.extend((i,v) for i,v in collector.feed(row) if first<i<=last)
    result=replay(events)
    result.update(axis=axis,event_start=first,event_end=last,expected_events=last-first,
                  complete=len(events)==last-first and result['chain_discontinuities']==0,
                  session_trace_gaps=collector.gaps,session_restarts=collector.restarts,crc_errors=parser.bad_crc,
                  source=str(trial),event_stream=events)
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('trial',type=Path)
    ap.add_argument('--axis',type=int,choices=(1,2),default=1);args=ap.parse_args()
    result=analyze(args.trial,args.axis);out=args.trial.with_name(args.trial.stem+'_trace_review.json')
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='event_stream'},indent=2))
