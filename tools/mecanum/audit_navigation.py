"""Synthesis release gate. Does not substitute for routed timing/CDC signoff."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def current_source_hash(relative, manifest):
    data = (ROOT/relative).read_bytes()
    if relative == 'fpga/mecanum/system_mecanum_top.sv':
        # Match only the build tool's two explicit parameter overrides. Keep
        # all remaining bytes, including line endings, under the hash check.
        data = re.sub(rb'\bLIDAR_RAW=[01]\b', b'LIDAR_RAW='+str(int(manifest['lidar_mode']=='raw_points')).encode(), data)
        data = re.sub(rb'\bNAVIGATION=[01]\b', b'NAVIGATION='+str(int(manifest['mecanum_navigation'])).encode(), data)
    return hashlib.sha256(data).hexdigest()


def main():
    folder = Path(sys.argv[1]).resolve()
    manifest = json.loads((folder/'manifest.json').read_text())
    assert manifest['mecanum_navigation']
    frozen = all(sha(folder/'source'/p) == h for p,h in manifest['source_sha256'].items())
    current = all(current_source_hash(p, manifest) == h for p,h in manifest['source_sha256'].items())
    assert frozen and current, 'RTL changed after freeze: repeat affected checks'
    netpath = folder/'impl/gwsynthesis/sensor.vg'
    net = netpath.read_text(errors='replace')
    bodies = dict(re.findall(r'\bmodule\s+(\w+)\s*\((.*?)endmodule', net, re.S))
    def primitives(name):
        return dict(Counter(re.findall(r'\b(SDPB|SDPX9B|DPB|SP|MULT\w*)\s+[^\s(]+\s*\(', bodies[name])))
    cells = {n:primitives(n) for n in ('vision_frame_stream','mc_nav_grid','mc_nav_raymap','mc_nav_sweep','nav_histogram')}
    assert cells['vision_frame_stream'].get('SDPB') == 129
    assert cells['mc_nav_grid'] == {'SDPB':4}, 'map did not infer expected BSRAM'
    assert cells['mc_nav_raymap'].get('SDPX9B') == 5, 'point FIFO lost synchronous block RAM inference'
    assert cells['nav_histogram'].get('SDPB')==4, 'histogram FIFO/mirrored sectors lost SDP inference'
    # Verify elaboration removed the unused arithmetic, rather than merely
    # checking that its opcode is rejected at the front of a full datapath.
    specialized={}
    for owner,instance,mask,forbidden in (
            ('mc_nav_path_target','math','00111',('cx','cy','cz')),
            ('mc_nav_system','scalar','11010',('root','rootbit','multiplier'))):
        kind=re.search(r'\b(\w+) '+instance+r'\s*\(',bodies[owner])
        assert kind, (owner,instance)
        body=bodies[kind[1]]
        registers=re.findall(r'\bDFF\w*\s+(\w+)\s*\(',body)
        remaining=[r for r in registers if any(re.match(stem+r'_\d+',r) for stem in forbidden)]
        assert not remaining, ('unused math datapath was retained',owner,remaining)
        specialized[owner]=dict(module=kind[1],operation_mask=mask,registers=len(registers),unused_registers=remaining)
    # A syntactic connectivity count highlights candidates for the physical
    # fanout report; it is deliberately not called routed fanout signoff.
    fanout = {}
    for name,body in bodies.items():
        if name.startswith('mc_nav_'):
            connections=Counter(re.findall(r'\.[A-Z][A-Z0-9_]*\(([^()]+)\)', body))
            fanout[name]=[(n,c) for n,c in connections.most_common(12) if n not in ('GND','VCC')]
    report=(folder/'impl/gwsynthesis/sensor_syn.rpt.html').read_text()
    logpath=folder/'synthesis.log'
    if not logpath.exists():logpath=folder/'build.log'
    synthesis_log=logpath.read_text(errors='replace')
    assert 'GowinSynthesis finish' in synthesis_log
    assert not any('WARN' in line and 'EX3638' in line and 'mc_nav_' in line for line in synthesis_log.splitlines()), 'implicit navigation declaration'
    usage={}
    for label in ('Logic','Register','BSRAM'):
        m=re.search(r'<td class="label">'+label+r'</td>\s*<td>(.*?)</td>',report,re.S)
        assert m,label
        usage[label]=re.sub(r'\s+',' ',m[1])
    dsp_macros=dict(re.findall(r'(MULT\w+)</td>\s*<td>(\d+)',report))
    latch=re.search(r'--Register as Latch</td>\s*<td>(\d+)',report)
    assert latch and int(latch[1])==0, 'unexpected inferred latch'
    unsupported=[]
    for name,body in bodies.items():
        for instance in re.findall(r'\bSP\s+(\S+)\s*\(',body):
            mode=re.search(r'defparam '+re.escape(instance)+r'\.WRITE_MODE=([^;]+)',body)
            if mode and mode[1]=="2'b10":unsupported.append(dict(module=name,instance=instance))
    assert not unsupported, ('GW5AT unsupported SP read-first WRITE_MODE=10',unsupported)
    rejection_path=folder/'incremental_rejection.json'
    rejection=json.loads(rejection_path.read_text()) if rejection_path.exists() else None
    if rejection:assert not rejection['pnr_started'] and not rejection['full_rebuild_fallback']
    else:assert manifest['incremental_pnr']['explicit_full_rebuild'], 'No explicit whole-system build authorization'
    result=dict(synthesis_gate_passed=True,release_to_hardware=False,
                source_frozen_unchanged=frozen,current_sources_match=current,
                intentional_top_overrides=dict(LIDAR_RAW=1,NAVIGATION=1),
                netlist_sha256=sha(netpath),usage=usage,dsp_macros_not_routed_block_count=dsp_macros,primitive_mapping=cells,
                math_specialization=specialized,
                module_connection_count_not_routed_fanout=fanout,
                implementation_blocker=rejection,
                unsupported_sp_write_modes=unsupported,
                pending=['routed DSP/BSRAM resource audit','congestion and placement hotspots',
                         'CDC setup/hold and exception matching','unconstrained paths',
                         'final setup/hold/MPW','SRAM candidate and physical calibration/acceptance'],
                programmed=False,flash_written=False,hardware_tested=False)
    (folder/'navigation_synthesis_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='module_connection_count_not_routed_fanout'},indent=2))


if __name__ == '__main__':
    main()
