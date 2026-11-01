"""Reroute an immutable M2 synthesis snapshot without repeating synthesis."""
import argparse,hashlib,json,re,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('parent',type=Path);ap.add_argument('--place',type=int,default=2);ap.add_argument('--route',type=int,default=0);a=ap.parse_args()
 parent=a.parent.resolve();meta=json.loads((parent/'manifest.json').read_text());stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
 work=parent.parent/(stamp+'_p'+str(a.place)+'r'+str(a.route));work.mkdir()
 for n in ('source','inputs'):shutil.copytree(parent/n,work/n)
 shutil.copytree(parent/'impl/gwsynthesis',work/'impl/gwsynthesis')
 for n in ('task4.cst','task4.sdc','task4.gprj','pin_audit.json','validation.json','full_top_regression.log'):shutil.copy2(parent/n,work/n)
 vg=work/'impl/gwsynthesis/task4.vg';frozen=sha(vg)
 # Gowin re-elaborates relocated projects. Restore the reviewed netlist after
 # that bookkeeping synthesis, as in the validated control50 comparison flow.
 tcl=(parent/'build.tcl').read_text().replace('run all','run syn\nfile copy -force {'+(parent/'impl/gwsynthesis/task4.vg').as_posix()+'} {'+vg.as_posix()+'}\nrun pnr')
 tcl=re.sub(r'set_option -place_option \d+','set_option -place_option '+str(a.place),tcl)
 tcl=re.sub(r'set_option -route_option \d+','set_option -route_option '+str(a.route),tcl)
 (work/'build.tcl').write_text(tcl)
 meta.update(parent_build=str(parent),place_option=a.place,route_option=a.route,
             frozen_netlist_sha256=frozen,retry_tool_sha256=sha(Path(__file__)),mode='pnr',programmed=False,hardware_tested=False)
 (work/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n');print(work,flush=True)
 start=time.monotonic();cleaned=False;timeout=False;complete_at=None
 with (work/'build.log').open('w',encoding='utf-8') as log:
  proc=subprocess.Popen(['C:/Gowin/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe',str(work/'build.tcl')],cwd=work,stdout=log,stderr=subprocess.STDOUT)
  while proc.poll() is None:
   time.sleep(1);text=(work/'build.log').read_text(encoding='utf-8',errors='replace')
   if '[100%] Power analysis completed' in text and (work/'impl/pnr/task4.fs').exists():
    if complete_at is None:complete_at=time.monotonic()
    if time.monotonic()-complete_at>30:proc.terminate();proc.wait(timeout=15);cleaned=True;break
   if time.monotonic()-start>3600:proc.terminate();proc.wait(timeout=15);timeout=True;break
 text=(work/'build.log').read_text(encoding='utf-8',errors='replace')
 meta.update(exit_code=proc.returncode,elapsed_s=time.monotonic()-start,timed_out=timeout,
  cli_terminated_after_completed_outputs=cleaned,tool_flow_completed='[100%] Power analysis completed' in text and (work/'impl/pnr/task4.fs').exists())
 meta['sources_unchanged']=all(sha(ROOT/p)==h for p,h in {**meta['source_sha256'],**meta['constraint_inputs_sha256']}.items()) and sha(vg)==frozen
 if meta['tool_flow_completed']:
  audits={}
  for name in ('control50','lk_state'):
   out=work/(name+'_audit.json');p=subprocess.run([sys.executable,str(ROOT/'tools'/('audit_'+name+'.py')),'--netlist',str(vg),'--output',str(out)],cwd=ROOT,capture_output=True,text=True)
   (work/(name+'_audit.log')).write_text(p.stdout+p.stderr);audits[name]=p.returncode==0 and out.exists() and json.loads(out.read_text())['passed']
  meta['netlist_audits']=audits
  p=subprocess.run([sys.executable,str(ROOT/'tools/audit_control50_timing.py'),str(work),'--basename','task4'],cwd=ROOT,capture_output=True,text=True)
  (work/'timing_audit.log').write_text(p.stdout+p.stderr)
  meta['cdc_timing_audit_passed']=p.returncode==0 and (work/'timing_audit.json').exists() and json.loads((work/'timing_audit.json').read_text())['passed']
  tr=(work/'impl/pnr/task4.tr').read_text(errors='replace')
  meta['violations']={k.lower():int(re.search(r'<Numbers of '+k+r' Violated Endpoints>:(\d+)',tr)[1]) for k in ('Setup','Hold')}
  meta['min_pulse_width_slack_ns']=min(float(v) for v in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',tr))
  meta['pulse_width_passed']=meta['min_pulse_width_slack_ns']>=0
  meta['clock_route_warnings']=[s for s in (work/'impl/pnr/task4.log').read_text(errors='replace').splitlines() if 'PR1014' in s]
  meta['clock_routing_passed']=not meta['clock_route_warnings']
  meta['bitstream_sha256']=sha(work/'impl/pnr/task4.fs');meta['netlist_sha256']=sha(vg)
  meta['timing_sha256']=sha(work/'impl/pnr/task4.tr')
 meta['passed']=meta['tool_flow_completed'] and meta['sources_unchanged'] and all(meta.get('netlist_audits',{'missing':False}).values()) and meta.get('cdc_timing_audit_passed',False) and meta.get('pulse_width_passed',False) and meta.get('clock_routing_passed',False)
 (work/'result.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps({k:v for k,v in meta.items() if k not in ('source_sha256','constraint_inputs_sha256')},indent=2),flush=True)
if __name__=='__main__':main()
