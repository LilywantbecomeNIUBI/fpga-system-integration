"""Publish the exact experimental SRAM image and evidence on a separate branch."""
import argparse,gzip,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('build',type=Path)
    args=parser.parse_args();src=args.build.resolve()
    meta=json.loads((src/'result.json').read_text());test=json.loads((src/'test_result.json').read_text())
    active=json.loads((ROOT/'runs/commissioning/active_system_sram.json').read_text())
    assert active['build']==str(src) and active['download_confirmed'] and active['experimental']
    assert meta['passed'] is False and test['passed'] and test['source_sha256']==meta['source_sha256']
    assert all(sha(src/'source'/p)==h for p,h in meta['source_sha256'].items())
    bit=src/'impl/pnr/sensor.fs';assert sha(bit)==meta['bitstream_sha256']==active['bitstream_sha256']
    name='experimental-360-routing-20261102';branch='codex/'+name
    out=ROOT/'releases'/name;assert not out.exists();out.mkdir()
    shutil.copytree(src/'source',out/'source')
    for file in ('sensor.gprj','sensor.cst','sensor.sdc','build.tcl','manifest.json','pin_audit.json',
                 'result.json','test_result.json','tb_system_sector_ethernet.sv','timing_audit.json',
                 'routing_diagnosis.json','frame_bank_netlist_audit.json','control50_audit.json','lk_state_audit.json'):
        shutil.copy2(src/file,out/file)
    for file in ('sensor.rpt.txt','sensor.tr'):shutil.copy2(src/'impl/pnr'/file,out/file)
    (out/'firmware.fs.gz').write_bytes(gzip.compress(bit.read_bytes(),mtime=0))
    board=Path(active['last_program']).parent
    shutil.copy2(board/'result.json',out/'board_program.json')
    shutil.copy2(board/'programmer.log',out/'programmer.log')
    shutil.copy2(board/'previous_active_system_sram.json',out/'previous_board_image.json')
    shutil.copy2(src/'live_observation.json',out/'live_observation.json')
    shutil.copy2(ROOT/'docs/ROUTING_OPTIMIZATION_20261102.md',out/'ROUTING_OPTIMIZATION.md')
    (out/'README.md').write_text('''# 360 点整机实验版：已下载 SRAM，未通过时序验收

本目录包含实际下载的冻结源码、独立 Gowin 工程、压缩位流、时序报告、仿真及下载证据。
相机、光流、导航、IMU、360 点雷达、网口状态上传和原串口命令入口包含在整机中。

**实验状态：setup 881、hold 0，最小脉宽裕量 −0.849 ns，PR1014 仍存在。不得标为时序通过或实板功能验证通过。**
JTAG SRAM 下载完成，未写 Flash。下载后 PC 有线链路断开，未能完成 FPGA/Nano 网口验收。
该整机电机输出不再固定为 GND；仿真中未 ARM 时输出低，下载过程未发送任何串口控制命令。

优化内容：4 Bank RAM 写命令流水、资源复制、place=2/route=0、时钟资源清理和每域详细时序报告。
完整顶层仿真通过，BRAM 总量仍为 236，寄存器增加 288。全点雷达源码完整保留在 source 中。

位流 SHA256：`'''+meta['bitstream_sha256']+'''`。
`firmware.fs.gz` 是无损 gzip 压缩，用 Python `gzip.decompress` 解压后应核对上述哈希。
Gowin V1.9.12.04 打开 `sensor.gprj`，或执行 `gw_sh build.tcl` 可重新构建；重新构建可能得到不同布局。

恢复稳定版：标签 `lidar360-mpu6500-20261102`；原始稳定标签 `lidar360-static-20261102` 保持不变。
详细比较及后续瓶颈见 `ROUTING_OPTIMIZATION.md`。冻结仿真台中的本机夹具路径保留作证据。
''',encoding='utf-8')
    (out/'SHA256.json').write_text(json.dumps({p.relative_to(out).as_posix():sha(p) for p in out.rglob('*') if p.is_file()},indent=2))
    selected=['fpga/navigation/system_task4_top.sv','fpga/vision/src/network_runtime_dual.v',
        'fpga/vision/src/vision/vision_frame_stream.v','tools/task4/build_ethernet.py',
        'tools/audit_control50_timing.py','tools/test_perception.py','tests/vision/tb_frame_banks.v',
        'tools/task4/report_route_trial.py','tools/task4/program_experimental_system.py',
        'tools/task4/publish_experimental_360.py','docs/ROUTING_OPTIMIZATION_20261102.md',
        'tools/task4/test_sector_system.py','tools/task4/test_raw_ethernet.py',
        'tests/navigation/tb_system_raw_ethernet.sv']
    for path in selected:
        if path in meta['source_sha256']:assert sha(ROOT/path)==meta['source_sha256'][path]
    index=ROOT/'build'/('publish-experimental-'+str(time.time_ns())+'.index')
    env=dict(os.environ,GIT_INDEX_FILE=str(index),GIT_TERMINAL_PROMPT='0')
    def git(*args,input=None):
        return subprocess.run(['git']+list(args),cwd=ROOT,env=env,input=input,text=True,
            capture_output=True,check=True,timeout=180).stdout.strip()
    parent=git('rev-parse','HEAD');working_branch=git('branch','--show-current')
    assert not git('ls-remote','--heads','origin',branch) and not git('ls-remote','--tags','origin',name)
    git('read-tree',parent);git('add','--',*selected);git('add','-f','--','releases/'+name)
    tree=git('write-tree')
    commit=git('commit-tree',tree,'-p',parent,input='Record experimental 360-point full-system routing trial and SRAM download\n\nBank frame RAM writes and enable resource replication. Preserve explicit timing failures (881 setup, negative pulse width, PR1014); this is not a stable release. Include the exact downloaded image and source snapshot.\n')
    git('update-ref','refs/heads/'+branch,commit,'0'*40)
    git('tag','-a',name,commit,'-m','Experimental SRAM download; timing NOT passed; network acceptance unavailable. Stable 360-point tags preserved.')
    git('push','--atomic','origin','refs/heads/'+branch+':refs/heads/'+branch,'refs/tags/'+name+':refs/tags/'+name)
    assert git('ls-remote','--heads','origin',branch).split()[0]==commit
    assert git('branch','--show-current')==working_branch
    result=dict(commit=commit,branch=branch,tag=name,parent=parent,working_branch=working_branch,
                release=str(out),timing_signoff=False,programmed_sram=True,flash_written=False)
    (src/'github_publish.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
