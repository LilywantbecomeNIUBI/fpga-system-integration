"""Publish two immutable timing checkpoints without switching the shared checkout."""
import gzip,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'build/lidar_ethernet_20261102/system'
PARENT='df4e936c3c6ca3aa19e05973340a2281ceb15cd7'
BRANCH='codex/timing-closure-20261103'
CHECKPOINTS=[('frame-read-pipeline-20261103','20261102T155742Z',550),
             ('timing-refactor-20261103','20261102T172152Z',6)]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def freeze(name,stamp,count):
    src=BASE/stamp;out=ROOT/'releases'/name
    meta=json.loads((src/'result.json').read_text());test=json.loads((src/'test_result.json').read_text())
    assert not meta['passed'] and meta['violations']==dict(setup=count,hold=0)
    assert not meta['programmed'] and test['passed'] and test['source_sha256']==meta['source_sha256']
    assert all(sha(src/'source'/p)==h for p,h in meta['source_sha256'].items())
    assert sha(src/'impl/pnr/sensor.fs')==meta['bitstream_sha256']
    if out.exists():
        # Resume a publication after a pre-push validation failure without
        # rewriting an already-complete frozen package.
        files=json.loads((out/'SHA256.json').read_text())
        assert all(sha(out/p)==h for p,h in files.items())
        assert hashlib.sha256(gzip.decompress((out/'firmware.fs.gz').read_bytes())).hexdigest()==meta['bitstream_sha256']
        return out
    out.mkdir(parents=True)
    # core.autocrlf is enabled on this machine. Frozen evidence must keep its
    # original bytes on every platform, including legacy GBK RTL comments.
    (out/'.gitattributes').write_text('* -text\n',encoding='ascii')
    shutil.copytree(src/'source',out/'source')
    for file in ['sensor.gprj','sensor.cst','sensor.sdc','build.tcl','manifest.json','pin_audit.json',
                 'result.json','test_result.json','tb_system_sector_ethernet.sv','control50_audit.json',
                 'lk_state_audit.json','timing_audit.json','frame_read_timing_audit.json','routing_diagnosis.json',
                 'frame_read_netlist_audit.json','offline_verification.json','remaining_timing_paths.json',
                 'working_tree_formatting.json','local_controls_netlist_audit.json']:
        if (src/file).exists():shutil.copy2(src/file,out/file)
    shutil.copy2(src/'impl/pnr/sensor.rpt.txt',out/'sensor.rpt.txt')
    artifacts={}
    for original,compressed in [('impl/pnr/sensor.fs','firmware.fs.gz'),
                                ('impl/pnr/sensor.tr','sensor.tr.gz'),
                                ('impl/gwsynthesis/sensor.vg','sensor.vg.gz')]:
        source=src/original;data=source.read_bytes()
        (out/compressed).write_bytes(gzip.compress(data,mtime=0))
        artifacts[compressed]=dict(original=original,original_sha256=sha(source),original_bytes=len(data))
    (out/'artifacts.json').write_text(json.dumps(artifacts,indent=2))
    fixtures=out/'fixtures';fixtures.mkdir()
    for file in ['input.hex','input.len','set_endpoint.hex','arp_endpoint.hex']:
        shutil.copy2(ROOT/'build/lidar_raw_ethernet_20261102'/file,fixtures/file)
    for file in ['tests/ethernet_pll_model.v','tests/imu/mpu6050_model.sv']:
        shutil.copy2(ROOT/file,fixtures/Path(file).name)
    document='FRAME_READ_PIPELINE_REVIEW_20261102.md' if count==550 else 'TIMING_ROOT_CAUSES_20261103.md'
    shutil.copy2(ROOT/'docs'/document,out/'REVIEW.md')
    (out/'README.md').write_text(f'''# {name}

**离线整机实验版本，未通过整机时序验收，未下载到板子。**

- 顶层 `system_task4_top`，360 点雷达、MPU6500/6050、图传、光流、导航与网口状态上传。
- 全点雷达实现保留在 `source/`；本冻结构建选择 360 点。
- 工具 setup 统计 {count}，hold 0；MPW {meta['min_pulse_width_slack_ns']:+.3f} ns，PR1014 仍存在。
- 冻结源码完整顶层仿真通过；仿真强制 PHY 初始化并注入视频，不代表实板启动通过。
- 本整机有电机命令接收入口，输出并非静态固定 GND；仿真检查了未 ARM 时输出低。本次没有发送控制命令或进行下载。

独立 Gowin V1.9.12.04 工程：`sensor.gprj`；构建脚本：`build.tcl`，只构建不下载。
原位流、时序详报、综合网表分别无损压缩为 `firmware.fs.gz`、`sensor.tr.gz`、`sensor.vg.gz`。
可用 Python `gzip.decompress` 解压，原文件哈希见 `artifacts.json`，包内文件哈希见 `SHA256.json`。
原位流 SHA256：`{meta['bitstream_sha256']}`。重新构建可能获得不同版图，必须重新验收。

`source/` 是当时实际编译的完整输入。目录内 `.gitattributes` 禁止 Git 换行转换，保持哈希一致。
`fixtures/` 保存冻结仿真台所需输入和模型；原台中的 `build/lidar_raw_ethernet_20261102` 相对路径保留。
工作树中的格式清理（如有）单独记录，不修改冻结编译输入。

稳定传感器恢复版本：`lidar360-mpu6500-20261102`。它只提供 360 点雷达和状态采集，不含图传/光流。
详细改动、已验证范围和残余问题见 `REVIEW.md`。
''',encoding='utf-8')
    (out/'SHA256.json').write_text(json.dumps({p.relative_to(out).as_posix():sha(p)
        for p in sorted(out.rglob('*')) if p.is_file()},indent=2))
    return out

def main():
    index=ROOT/'build'/('publish-timing-'+str(time.time_ns())+'.index')
    env=dict(os.environ,GIT_INDEX_FILE=str(index),GIT_TERMINAL_PROMPT='0')
    def git(*args,data=None):
        r=subprocess.run(['git']+list(args),cwd=ROOT,env=env,input=data,capture_output=True,check=True,timeout=180)
        return r.stdout
    def txt(*args):return git(*args).decode().strip()
    branch_before=txt('branch','--show-current')
    assert not txt('ls-remote','--heads','origin',BRANCH)
    for name,_,_ in CHECKPOINTS:assert not txt('ls-remote','--tags','origin',name)
    assert txt('rev-parse','experimental-360-routing-20261102^{}')==PARENT
    outs=[freeze(*checkpoint) for checkpoint in CHECKPOINTS]
    # Check the shipped reference bytes directly. Do not rerun tests against a
    # shared worktree that may already contain the next debugging iteration.
    reference=json.loads((outs[0]/'manifest.json').read_text())['source_sha256']
    for name in ('fpga/vision/src/vision/vision_v23_bridge.v',
                 'fpga/vision/src/vision/vision_result_core.v','fpga/vision/src/optical_flow/flow_event_bridge.v'):
        assert sha(outs[0]/'source'/name)==reference[name]
    first_tag=CHECKPOINTS[0][0];last_tag=CHECKPOINTS[1][0]
    git('read-tree',PARENT)
    frame='fpga/vision/src/vision/vision_frame_stream.v'
    blob=git('hash-object','-w','--stdin','--path='+frame,
             data=(BASE/CHECKPOINTS[0][1]/'source'/frame).read_bytes()).decode().strip()
    git('update-index','--add','--cacheinfo','100644',blob,frame)
    git('add','--','docs/FRAME_READ_PIPELINE_REVIEW_20261102.md','tools/task4/audit_frame_read_timing.py')
    git('add','-f','--','releases/'+first_tag)
    tree=txt('write-tree')
    first=git('commit-tree',tree,'-p',PARENT,data=b'Pipeline frame RAM reads within each bank\n\nPreserve the routed 550-violation checkpoint. Bank output and cached_word setup pass; full-system timing does not. No hardware download.\n').decode().strip()
    rtl=['fpga/imu/mpu6050_reader.v','fpga/navigation/lidar_scan_360_source.sv','fpga/navigation/nav_vfh.sv',
         'fpga/vision/src/network_runtime_dual.v','fpga/vision/src/optical_flow/flow_event_bridge.v',
         'fpga/vision/src/vision/vision_frame_stream.v','fpga/vision/src/vision/vision_result_core.v',
         'fpga/vision/src/vision/vision_v23_bridge.v']
    frozen=BASE/CHECKPOINTS[1][1]
    selected=['docs/TIMING_ROOT_CAUSES_20261103.md','docs/FIRMWARE_VERSIONS.md',
        'tools/task4/publish_timing_checkpoints.py','tools/task4/build_ethernet.py','tools/audit_control50_timing.py',
        'tools/task4/report_route_trial.py','tools/task4/test_timing_refactor.py','tools/task4/test_network_legacy.py',
        'tools/task4/test_sector_system.py','tools/test_perception.py','tests/vision/tb_frame_banks.v',
        'tests/vision/tb_flow_elastic.v','tests/vision/tb_bridge_timing_equivalence.v',
        'tests/vision/tb_event_ingress_bursts.v','tests/vision/tb_rx_length_boundaries.v','tests/imu/tb_stale_deadline.sv']
    git('read-tree',first)
    # Stage exact tested RTL from the frozen build, not newer concurrent edits.
    # Strip only trailing spaces for readable root source; raw compiled bytes
    # remain unchanged inside the release directory with their original hash.
    for name in rtl:
        raw=(frozen/'source'/name).read_bytes().replace(b'\r\n',b'\n')
        clean=b'\n'.join(line.rstrip(b' \t') for line in raw.split(b'\n'))
        blob=git('hash-object','-w','--stdin','--path='+name,data=clean).decode().strip()
        git('update-index','--add','--cacheinfo','100644',blob,name)
    git('add','--',*selected);git('add','-f','--','releases/'+last_tag)
    tree=txt('write-tree')
    last=git('commit-tree',tree,'-p',first,data=b'Refactor timing-critical counters, event queues and dispatch words\n\nReduce reported setup violations from 550 to 6; sys50 and RX125 reach zero. Preserve the four data setup and two reset-release failures, negative MPW and PR1014. Freeze exact RTL, image and validation evidence. No download.\n').decode().strip()
    # Verify every committed archive byte before publishing, independent of
    # local core.autocrlf and the normal worktree index.
    for commit,out in zip([first,last],outs):
        for p in out.rglob('*'):
            if p.is_file():assert hashlib.sha256(git('show',commit+':'+p.relative_to(ROOT).as_posix())).hexdigest()==sha(p),p
    git('update-ref','refs/heads/'+BRANCH,last,'0'*40)
    for tag,commit in [(first_tag,first),(last_tag,last)]:
        git('tag','-a',tag,commit,'-m','Experimental offline checkpoint; full timing NOT passed; no download. Stable sensor tags unchanged.')
    plan=dict(branch=BRANCH,checkpoints={first_tag:first,last_tag:last},parent=PARENT,working_branch=branch_before,
              programmed=False,full_timing_passed=False)
    (BASE/CHECKPOINTS[1][1]/'git_publish_plan.json').write_text(json.dumps(plan,indent=2))
    refs=['refs/heads/'+BRANCH]+['refs/tags/'+name for name,_,_ in CHECKPOINTS]
    git('push','--atomic','origin',*[ref+':'+ref for ref in refs])
    assert txt('ls-remote','--heads','origin',BRANCH).split()[0]==last
    for tag,commit in plan['checkpoints'].items():assert txt('ls-remote','origin','refs/tags/'+tag+'^{}').split()[0]==commit
    assert txt('branch','--show-current')==branch_before
    (BASE/CHECKPOINTS[1][1]/'github_publish.json').write_text(json.dumps(plan,indent=2))
    print(json.dumps(plan,indent=2))

if __name__=='__main__':main()
