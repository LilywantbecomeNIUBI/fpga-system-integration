"""Run real wire fixtures against new and frozen network RTL on every clock."""
from pathlib import Path
import hashlib,re
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/tests/network_cycle';OUT.mkdir(parents=True,exist_ok=True)
reference=ROOT/'tests/mecanum/reference/network_360_verified.v'
assert hashlib.sha256(reference.read_bytes()).hexdigest()=='28c4c01c562903759751459662dc843fdf6a119d07f6b46153de286e9a8975a7'
ref=OUT/'network_reference.v'
ref.write_text(reference.read_text().replace('network_runtime_dual','network_runtime_dual_reference').replace('network_dispatch_word','network_dispatch_word_reference'))
bench=(ROOT/'tests/vision/tb_network_dual.v').read_text()
start=bench.index('    network_runtime_dual #');end=bench.index('    always @(posedge clk)',start)
instance=bench[start:end].replace('network_runtime_dual #','network_runtime_dual_reference #').replace(' dut(',' reference_dut(')
signals={'video_read':'read','video_done':'done','tx_data':'tx','tx_valid':'en',
 'target_ip':'target','target_port':'port','cache_hit':'hit','target_mac':'mac','enabled':'enabled',
 'target2_ip':'target2','target2_port':'port2','cache2_hit':'hit2','target2_mac':'mac2','enabled2':'enabled2'}
for port,wire in signals.items():instance=instance.replace('.%s(%s)'%(port,wire),'.%s(ref_%s)'%(port,wire))
declarations='''
 wire ref_read,ref_done,ref_en,ref_hit,ref_hit2,ref_enabled,ref_enabled2;
 wire [7:0] ref_tx;wire [31:0] ref_target,ref_target2;
 wire [15:0] ref_port,ref_port2;wire [47:0] ref_mac,ref_mac2;
'''
monitor='''
 always @(negedge clk)if(rst_n)begin
  if({ref_read,ref_done,ref_en,ref_target,ref_target2,ref_port,ref_port2,ref_hit,ref_hit2,ref_mac,ref_mac2,ref_enabled,ref_enabled2} !==
     {read,done,en,target,target2,port,port2,hit,hit2,mac,mac2,enabled,enabled2})$fatal(1,"public network cycle mismatch");
  if(en && ref_tx!==tx)$fatal(1,"TX byte/cycle mismatch");
  if({dut.retry_count,dut.retry_count2,dut.attempts,dut.attempts2,dut.arp_due0,dut.arp_due1} !==
     {reference_dut.retry_count,reference_dut.retry_count2,reference_dut.attempts,reference_dut.attempts2,reference_dut.arp_due0,reference_dut.arp_due1})$fatal(1,"retry/cache cycle mismatch");
  if(dut.retry_target_ip0!==target || dut.retry_target_ip1!==target2)$fatal(1,"local endpoint replica mismatch");
 end
'''
bench=bench[:end]+declarations+instance+monitor+bench[end:]
path=OUT/'tb_network_dual_cycle.v';path.write_text(bench)
original=ROOT/'vendor/vision/test_network_dual_original.py'
source=original.read_text()
source=source.replace('ROOT=Path(__file__).resolve().parents[1]','ROOT=Path('+repr(str(ROOT))+')')
source=source.replace("'sim/tb_network_dual.v'",repr(str(path))+','+repr(str(ref)))
source=source.replace("'src/","'fpga/vision/src/").replace('timeout=45','timeout=240')
exec(compile(source,str(original),'exec'),dict(__file__=str(original),__name__='__main__'))
