"""Audit physical navigation CDC coverage, constraints, fanout and congestion."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import re
import sys


def reset_path_tables(trace):
    """Unfiltered worst-path tables include generated synchronous reset release."""
    results={}
    for kind,end in (('Recovery','3.1.4 Removal Paths Table'),('Removal','3.2 Minimum Pulse Width Table')):
        start='3.1.'+('3' if kind=='Recovery' else '4')+' '+kind+' Paths Table'
        section=trace.rsplit('\n'+start,1)[1].split('\n'+end,1)[0]
        rows=re.findall(r'^\s+(\d+)\s+([-\d.]+)\s+(\S+)\s+(\S+)\s+',section,re.M)
        assert rows, 'Missing unfiltered '+kind+' report'
        results[kind.lower()]=dict(worst_slack_ns=min(float(r[1]) for r in rows),
            reported_paths=len(rows),negative_reported_paths=sum(float(r[1])<0 for r in rows),
            worst_source=rows[0][2],worst_sink=rows[0][3],
            scope='Unfiltered worst-path sample; sample count is not total endpoint coverage.')
    return results


def clock_inventory(net,trace):
    """Check every synthesized clock connection, including RAM/DSP clock vectors."""
    bodies=dict(re.findall(r'\bmodule (\w+) \((.*?)endmodule',net,re.S))
    allowed={'clk50m_d','clk50','clk125','camera_pclk_d','rgmii_rxc_d','GND','VCC'}
    rows={};unexpected={}
    for name,body in bodies.items():
        clocks=Counter(re.findall(r'\.(?:CLK|CLKA|CLKB|CLK0|CLK1)\(([^)]+)\)',body))
        if not clocks:continue
        rows[name]=dict(clocks)
        invalid=[c for c in clocks if not set(re.findall(r'\w+',c))<=allowed]
        if invalid:unexpected[name]=invalid
    section=trace.rsplit('\n2.2 Clock Summary',1)[1].split('\n2.3 Max Frequency Summary',1)[0]
    periods={n:float(p) for n,p in re.findall(r'^\s+\d+\s+(\w+)\s+(?:Base|Generated)\s+([\d.]+)',section,re.M)}
    expected={'sys50':20.,'cam_pclk':11.905,'eth_rx125':8.,'control50':20.,'eth125':8.}
    passed=bool(rows) and not unexpected and periods==expected
    return dict(passed=passed,clock_periods_ns=periods,unexpected_clock_connections=unexpected,
        module_clock_connections=rows,
        scope='Synthesized CLK/CLKA/CLKB/CLK0/CLK1 inventory plus active STA clock summary; constants on disabled primitive clock inputs are allowed. Not an external I/O timing contract.')


def audit(folder):
    folder=Path(folder).resolve();trace=(folder/'impl/pnr/sensor.tr').read_text(errors='replace')
    netpath=folder/'impl/gwsynthesis/sensor.vg';net=netpath.read_text(errors='replace')
    module=re.search(r'\b(\w+) navigation_safety_cdc \(',net)[1]
    body=re.search(r'\bmodule '+module+r' \(.*?endmodule',net,re.S)[0]
    cells=set(re.findall(r'\bDFF\w*\s+(dst_data_\w+)\s*\(',body))
    bits={int(re.search(r'dst_data_(\d+)',c)[1]) for c in cells}
    assert bits==set(range(20))|set(range(32,64)), ('navigation mailbox payload optimized unexpectedly',bits)
    exception=trace[trace.rfind('\n3.7 Timing Exceptions Report'):trace.rfind('\n3.8 Timing Constraints Report')]
    rows=[]
    for block in exception.split('Path Summary:')[1:]:
        source=re.search(r'^From\s+: (\S+)',block,re.M);sink=re.search(r'^To\s+: (\S+)',block,re.M)
        if not source or not sink or '/navigation_safety_cdc/dst_data_' not in sink[1]:continue
        delay=re.search(r'Arrival Data Path Delay: \(cell: ([\d.]+).*?route: ([\d.]+).*?tC2Q: ([\d.]+)',block,re.S)
        slack=re.search(r'^Slack\s+: ([-\d.]+)',block,re.M)
        if not delay or not slack:continue
        rows.append(dict(source=source[1],sink=sink[1],slack_ns=float(slack[1]),data_delay_ns=round(sum(map(float,delay.groups())),3)))
    observed={r['sink'].split('/')[-1] for r in rows}
    coverage=cells<=observed
    cdc_pass=coverage and bool(rows) and all('/navigation_safety_cdc/held_' in r['source'] and r['data_delay_ns']<10 and r['slack_ns']>=0 for r in rows)
    constraint_section=trace[trace.rfind('\n3.8 Timing Constraints Report'):]
    navigation_constraints=[line.strip() for line in constraint_section.splitlines() if 'navigation_safety' in line]
    constraints_active=bool(navigation_constraints) and all('Actived' in line for line in navigation_constraints)
    inactive_constraints=[line.strip() for line in constraint_section.splitlines() if re.match(r'\s+TC_\w+\s+',line) and 'Actived' not in line]
    clock_check=clock_inventory(net,trace)
    fstart=trace.rfind('\n3.5 High Fanout Nets Report');cstart=trace.rfind('\n3.6 Route Congestions Report')
    ftext=trace[fstart:cstart];ctext=trace[cstart:trace.rfind('\n3.7 Timing Exceptions Report')]
    fanout=[dict(fanout=int(n),net=name,worst_slack_ns=float(slack),max_delay_ns=float(delay)) for n,name,slack,delay in re.findall(r'^\s+(\d+)\s+(\S+)\s+([-\d.]+)\s+([-\d.]+)\s*$',ftext,re.M)]
    congestion=[dict(grid=loc,occupancy=float(value)) for loc,value in re.findall(r'^\s+(R\d+C\d+)\s+([\d.]+)\s*$',ctext,re.M)]
    assert fanout and congestion, 'Missing physical fanout/congestion reports'
    counts={kind.lower():int(re.search(r'<Numbers of '+kind+r' Violated Endpoints>:(\d+)',trace)[1]) for kind in ('Setup','Hold')}
    reset_tables=reset_path_tables(trace)
    reset_pass=all(r['worst_slack_ns']>=0 for r in reset_tables.values())
    result=dict(passed=cdc_pass and constraints_active and not inactive_constraints and clock_check['passed'] and counts==dict(setup=0,hold=0) and reset_pass,
                navigation_cdc_passed=cdc_pass,constraint_activation_passed=constraints_active,
                reset_release_passed=reset_pass,reset_path_tables=reset_tables,
                inactive_constraints=inactive_constraints,clock_inventory=clock_check,
                expected_dynamic_payload_bits=sorted(bits),expected_capture_registers=len(cells),
                covered_capture_registers=len(cells&observed),reported_paths=len(rows),
                worst_cdc_data_delay_ns=max((r['data_delay_ns'] for r in rows),default=None),
                worst_cdc_slack_ns=min((r['slack_ns'] for r in rows),default=None),
                navigation_constraints=navigation_constraints,violations=counts,
                high_fanout=fanout,congestion=congestion,cdc_paths=rows,
                scope='Physical bundled-data setup and actual exception coverage; no false-path or clock-period changes.',
                limitations='Clocked internal setup/hold is checked globally; asynchronous motor/UART/LED outputs have no external synchronous I/O timing contract. This is not a claim that every physical I/O has an output delay.',
                netlist_sha256=hashlib.sha256(netpath.read_bytes()).hexdigest(),
                full_sta_signoff=False,programmed=False)
    (folder/'navigation_timing_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('cdc_paths','navigation_constraints','clock_inventory')},indent=2))
    return result


if __name__=='__main__':
    result=audit(sys.argv[1]);raise SystemExit(0 if result['passed'] else 1)
