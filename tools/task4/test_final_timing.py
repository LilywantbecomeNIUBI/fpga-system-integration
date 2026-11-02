"""Final timing refactor functional checks; frozen pre-change mailbox reference."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/tests/final_timing'
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    name='fpga/vision/src/vision/vision_mailbox.v'
    old=ROOT/'releases/timing-refactor-20261103/source'/name
    digest=hashlib.sha256(old.read_bytes()).hexdigest()
    assert digest=='0376dacbc4fe05a59e928f719fce302b9f70779aac5e61d1d20f1b540e15307a'
    ref=OUT/'mailbox_reference.v'
    ref.write_text(old.read_text().replace('module vision_mailbox','module vision_mailbox_reference',1))
    cases=[]
    network_old=ROOT/'tests/mecanum/reference/network_360_verified.v'
    network_digest=hashlib.sha256(network_old.read_bytes()).hexdigest()
    assert network_digest=='28c4c01c562903759751459662dc843fdf6a119d07f6b46153de286e9a8975a7'
    network_ref=OUT/'network_reference.v'
    network_ref.write_text(network_old.read_text().replace('network_runtime_dual','network_runtime_dual_reference').replace('network_dispatch_word','network_dispatch_word_reference'))
    for hz in (125000000,999,1):
        for cache in (30,1):
            cases.append((f'same_cycle_{hz}_{cache}','tb_retry_same_cycle',
                [f'-Ptb_retry_same_cycle.HZ={hz}',f'-Ptb_retry_same_cycle.CACHE={cache}'],[
                'tests/vision/tb_retry_same_cycle.v',str(network_ref),'fpga/vision/src/network_runtime_dual.v',
                'fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v',
                'fpga/vision/src/eth_udp_gmii/ip_checksum.v']))
    for hz in (125000000,999,1):
        cases.append((f'retry_{hz}','tb_retry_segments',[f'-Ptb_retry_segments.HZ={hz}'],[
            'tests/vision/tb_retry_segments.v','fpga/vision/src/network_runtime_dual.v',
            'fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v',
            'fpga/vision/src/eth_udp_gmii/ip_checksum.v']))
    for width in (257,256,1):
        cases.append((f'mailbox_{width}','tb_reset_mailbox_banks',[f'-Ptb_reset_mailbox_banks.WIDTH={width}'],[
            'tests/vision/tb_reset_mailbox_banks.v','fpga/vision/src/vision/vision_mailbox.v',
            'fpga/vision/src/reset_sync.v',str(ref)]))
    results={}
    for name,top,opts,files in cases:
        binary=OUT/(name+'.vvp')
        for cmd in (['iverilog','-g2012','-s',top,*opts,'-o',str(binary),*files],['vvp',str(binary)]):
            r=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=180)
            with (OUT/(name+'.log')).open('a') as log:log.write(r.stdout+r.stderr)
            if r.returncode:raise RuntimeError(r.stdout+r.stderr)
        print(r.stdout,flush=True,end='');results[name]=True
    (OUT/'result.json').write_text(json.dumps(dict(passed=True,results=results,reference_sha256=digest,
        verified360_network_sha256=network_digest),indent=2))
if __name__=='__main__':main()
