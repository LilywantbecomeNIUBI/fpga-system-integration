"""Create a new, verified offline release; never accesses a device or live runtime."""
from pathlib import Path
import argparse,ast,hashlib,json,shutil,subprocess,sys,zipfile
ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_file(source,target):
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT.parents[1]/'output/lidar_raw_ethernet_20261102')
    args=parser.parse_args();target=args.output.resolve()
    assert not target.exists() and not target.with_suffix('.zip').exists(),'Preserve existing releases; choose a new destination'
    base=ROOT/'build/lidar_raw_ethernet_20261102'
    static=base/'diagnostic/20261102T103622Z';system=base/'system/20261102T104503Z'
    meta=json.loads((static/'result.json').read_text());test=json.loads((static/'test_result.json').read_text())
    assert meta['passed'] and test['passed'] and meta['lidar_mode']=='raw_points'
    assert meta['violations']==dict(setup=0,hold=0) and meta['motor_outputs_constant_low']
    assert test['source_sha256']==meta['source_sha256'] and test['all_points_checked']==12000
    assert sha(static/'impl/pnr/sensor.fs')==meta['bitstream_sha256']
    for folder in (static,system):
        frozen=json.loads((folder/'manifest.json').read_text())
        assert all(sha(folder/'source'/p)==h for p,h in frozen['source_sha256'].items())
    assert sha(ROOT/test['testbench_path'])==test['testbench_sha256']
    for name in ('raw_regression','points_regression','receiver_benchmark'):
        audit=json.loads((base/(name+'.json')).read_text());assert audit['passed']
        assert all(sha(ROOT/p)==h for p,h in audit.get('source_sha256',{}).items())
    target.mkdir(parents=True)
    for source,name in ((static,'static_sensor_build'),(system,'full_system_candidate')):
        for item in source.rglob('*'):
            if not item.is_file() or item.suffix in ('.vvp','.user'):continue
            if 'impl' in item.parts and item.suffix not in ('.fs','.txt','.html','.tr','.vg'):continue
            copy_file(item,target/name/item.relative_to(source))
    candidate=target/'full_system_candidate'
    (candidate/'validation.json').write_text(json.dumps(dict(hierarchy_elaborated=True,pnr_completed=False,hardware_tested=False),indent=2))
    package=target/'nano_gateway'
    subprocess.run([sys.executable,str(ROOT/'tools/prepare_nano.py'),'--output',str(package)],cwd=ROOT,check=True)
    # Inspect the actual isolated package, not imports from the working checkout.
    isolated="import sys;sys.path.insert(0,sys.argv[1]);from nano.gateway import Gateway;g=Gateway(dict(telemetry_transport='udp',lidar_source='fpga'));assert not g.threads;print('PASS isolated Nano gateway import; no IO')"
    subprocess.run([sys.executable,'-I','-c',isolated,str(package)],cwd=target,check=True)
    manifest=json.loads((package/'MANIFEST.json').read_text())
    assert all(sha(package/p)==h for p,h in manifest['files'].items())
    pyfiles=list((package/'nano').rglob('*.py'))+list((package/'shared').rglob('*.py'))
    for path in pyfiles:ast.parse(path.read_text(encoding='utf-8'),feature_version=(3,6))
    evidence=('raw_regression.json','points_regression.json','receiver_benchmark.json','raw_recorded_frames.bin',
              'raw_compile.log','raw_sim.log','points_compile.log','points_sim.log','tx.hex','points_frames.hex','input.hex','input.len')
    for name in evidence:copy_file(base/name,target/'evidence'/name)
    review=['fpga/navigation/lidar_points_source.sv','fpga/navigation/system_task4_top.sv','fpga/diagnostics/m2_sensor_eth_diag_top.sv',
        'host/index.html','host/task4_console.html','host/task4_console.py','host/motor_protocol.py','nano/gateway.py',
        'shared/lidar_raw.py','shared/lidar_display.py','shared/streams.py','shared/telemetry_udp.py','tools/prepare_nano.py',
        'tools/task4/build.py','tools/task4/build_ethernet.py','tools/task4/protocol.py','tools/task4/verify_ethernet.py',
        'tools/task4/program_ethernet.py','tools/task4/prepare_live_ethernet.py','tools/task4/telemetry_endpoint.py',
        'tools/task4/test_raw_source.py','tools/task4/test_raw_ethernet.py','tools/task4/test_raw_ethernet_top.py',
        'tools/task4/test_ethernet.py','tools/task4/benchmark_raw_receiver.py','tools/task4/package_raw_ethernet.py',
        'tests/test_lidar_raw.py','tests/test_telemetry_throughput.py','tests/test_lidar_ui.js',
        'tests/navigation/tb_lidar_points.sv','tests/navigation/tb_lidar_raw_ethernet.sv',
        'tests/diagnostics/tb_m2_sensor_raw_ethernet.sv','tests/ethernet_pll_model.v','tests/imu/mpu6050_model.sv']
    for name in review:copy_file(ROOT/name,target/'source_review'/name)
    copy_file(ROOT/'docs/task4/LIDAR_RAW_ETHERNET_20261102.md',target/'方案与验证.md')
    (target/'README.md').write_text('''# STL-27L 全点以太网离线交付

此目录仅包含离线产物，没有下载或启动设备。旧 360 点交付保留。

- `static_sensor_build`：通过 PnR 和冻结顶层仿真的静态传感器 Gowin 工程/位流；十二路电机脚固定低，无相机、运动导航或命令接收。
- `full_system_candidate`：完整视觉/导航候选源码与约束，已层次展开，尚未完成该版 PnR。
- `nano_gateway`：Python 3.6 兼容网关、共享解码、配置样例及说明；已隔离导入，未部署。
- `source_review/host`：待部署的主机页面和代码，全量/720/360显示及自动范围。
- `evidence`：原始录制包夹具、全点逐点回归、FIFO测试和开发机吞吐测量；不代表硬件验收。
- `SHA256.json`：本目录全部文件的内容哈希（不含清单自身）。

详见《方案与验证.md》。状态用 UDP 6205，控制保留原 UART。
''',encoding='utf-8')
    hashes={p.relative_to(target).as_posix():sha(p) for p in target.rglob('*') if p.is_file()}
    (target/'SHA256.json').write_text(json.dumps(dict(programmed=False,deployed=False,hardware_tested=False,files=hashes),indent=2))
    archive=Path(shutil.make_archive(str(target),'zip',root_dir=target.parent,base_dir=target.name))
    with zipfile.ZipFile(archive) as zipped:assert zipped.testzip() is None
    assert all(sha(target/p)==h for p,h in hashes.items())
    result=dict(passed=True,output=str(target),archive=str(archive),archive_sha256=sha(archive),files=len(hashes),
                nano_files=len(manifest['files']),python36_modules=len(pyfiles),programmed=False,deployed=False)
    (base/'release_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
