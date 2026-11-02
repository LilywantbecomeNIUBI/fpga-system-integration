"""Summarize routed evidence without mistaking sampled paths for all endpoints."""
import argparse, collections, json, re
from pathlib import Path


def summarize(folder):
    trace=(folder/'impl/pnr/sensor.tr').read_text(errors='replace')
    report=(folder/'impl/pnr/sensor.rpt.txt').read_text(errors='replace')
    result={'folder':str(folder.resolve()),'clock_endpoints':{},'setup_reports':[],
            'clock_allocation':[], 'resources':{}}
    for clock,kind,tns,count in re.findall(r'^\s+(\w+)\s+(setup|hold)\s+([-\d.]+)\s+(\d+)\s*$',trace,re.M):
        result['clock_endpoints'].setdefault(clock,{})[kind]={'tns_ns':float(tns),'violated_endpoints':int(count)}
    setup=trace.rsplit('3.1.1 Setup Paths Table',1)[1].split('3.1.2 Hold Paths Table')[0]
    row_re=r'^\s+(\d+)\s+([-\d.]+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*$'
    for section in setup.split('<Report Command>:')[1:]:
        command=section.splitlines()[0]
        paths=[dict(number=int(n),slack_ns=float(s),source=a,sink=b,source_clock=c,destination_clock=d,
                    relation_ns=float(rel),skew_ns=float(skew),data_delay_ns=float(delay))
               for n,s,a,b,c,d,rel,skew,delay in re.findall(row_re,section,re.M)]
        groups=collections.Counter('/'.join(p['sink'].split('/')[:2]) if p['sink'].count('/')>=2
                                   else p['sink'].split('/')[0] for p in paths if p['slack_ns']<0)
        result['setup_reports'].append(dict(command=command,reported_paths=len(paths),
            sampled_failing_sink_groups=dict(groups),paths=paths))
    for net,tree,location in re.findall(r'^\s+(\S+)\s*\|\s*(PRIMARY|LW)\s*\|([^\n]+)',report,re.M):
        result['clock_allocation'].append(dict(net=net,tree=tree,location=location.strip()))
    for name in ('Logic','Register','BSRAM','DSP','PRIMARY','LW'):
        found=re.search(r'^\s*'+name+r'\s*\|([^\n]+)',report,re.M)
        if found:result['resources'][name]=found[1].strip()
    pulse=re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',trace)
    result['min_pulse_width_slack_ns']=min(map(float,pulse)) if pulse else None
    result['sample_note']='Each setup table is a worst-path sample limited by max_paths; group counts are not the full violated-endpoint distribution.'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--baseline',type=Path)
    args=parser.parse_args();result={'trial':summarize(args.folder)}
    if args.baseline:result['baseline']=summarize(args.baseline)
    target=args.folder/'routing_diagnosis.json';target.write_text(json.dumps(result,indent=2)+'\n')
    print(target)
    for label,data in result.items():
        print(label,json.dumps({k:data[k] for k in ('clock_endpoints','resources','min_pulse_width_slack_ns')}))


if __name__=='__main__':main()
