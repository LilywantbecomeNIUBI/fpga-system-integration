"""Distinguish filtered domain slack from global violations; audit CDC wires."""
import argparse,json,re
from pathlib import Path

def audit(work,basename='system'):
    text=(work/'impl/pnr'/(basename+'.tr')).read_text(errors='replace')
    table=text.rsplit('3.1.1 Setup Paths Table',1)[1].split('3.1.2 Hold Paths Table')[0]
    slacks={}
    for clock in ['control50','cam_pclk']:
        pattern=r'<Report Command>:report_timing -setup -from_clock \[get_clocks \{'+clock+r'\}\] -to_clock \[get_clocks \{'+clock+r'\}\].*?^\s+1\s+([-\d.]+)'
        slacks[clock]=float(re.search(pattern,table,re.S|re.M)[1])
    exceptions=text.split('3.7.1.1 Setup Analysis Report[1]')[-1].split('3.8 Timing Constraints Report')[0]
    rows=[]
    for block in exceptions.split('Path Summary:')[1:]:
        a=re.search(r'^From\s+: (\S+)',block,re.M);b=re.search(r'^To\s+: (\S+)',block,re.M)
        delay=re.search(r'Arrival Data Path Delay: \(cell: ([\d.]+).*?route: ([\d.]+).*?tC2Q: ([\d.]+)',block,re.S)
        if not (a and b and delay):continue
        rows.append(dict(source=a[1],sink=b[1],data_delay_ns=round(sum(map(float,delay.groups())),3),
                         slack_ns=float(re.search(r'^Slack\s+: ([-\d.]+)',block,re.M)[1])))
    gray=[]
    for stage in ['ws0','rs0']:
        points=[r for r in rows if re.search('/fifo/'+stage+r'_\d+_s\d+$',r['sink'])]
        assert {int(re.search(stage+r'_(\d+)',r['sink'])[1]) for r in points}==set(range(9)),(stage,points)
        assert len(points)==9,(stage,len(points))
        assert all(0<=r['data_delay_ns']<10 for r in points),points
        gray+=points
    mailboxes={}
    for name in ['flow_mailbox','observation_mailbox','seed_mailbox']:
        points=[r for r in rows if '/'+name+'/dst_data' in r['sink']]
        assert points,name
        assert all(r['data_delay_ns']<10 and r['slack_ns']>=0 for r in points),(name,points)
        mailboxes[name]=dict(paths=len(points),max_data_delay_ns=max(r['data_delay_ns'] for r in points))
    result=dict(passed=True,global_setup_violations=int(re.search(r'<Numbers of Setup Violated Endpoints>:(\d+)',text)[1]),
                global_hold_violations=int(re.search(r'<Numbers of Hold Violated Endpoints>:(\d+)',text)[1]),
                control50_wns_ns=slacks['control50'],cam_pclk_wns_ns=slacks['cam_pclk'],
                global_wns_ns=None,global_wns_note='Custom report_timing filters replace the default Setup table; do not label control50 slack as global WNS.',
                gray_paths=gray,max_gray_data_delay_ns=max(r['data_delay_ns'] for r in gray),mailboxes=mailboxes,
                cdc_data_budget_ns=10.0,timing_signoff=False)
    result['cdc_data_paths_passed']=True
    result['passed']=result['global_setup_violations']==result['global_hold_violations']==0
    (work/'timing_audit.json').write_text(json.dumps(result,indent=2)+'\n');return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('--basename',default='system');a=ap.parse_args();print(json.dumps(audit(a.work,a.basename),indent=2))
if __name__=='__main__':main()
