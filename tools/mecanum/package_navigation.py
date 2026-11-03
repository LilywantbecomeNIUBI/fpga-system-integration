"""Package reviewed source/evidence and an already audited SRAM candidate; never program hardware."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile
from audit_navigation import current_source_hash

ROOT=Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    folder=Path(sys.argv[1]).resolve()
    manifest=json.loads((folder/'manifest.json').read_text())
    synthesis=json.loads((folder/'navigation_synthesis_audit.json').read_text())
    integration=json.loads((folder/'system_test_result.json').read_text())
    kernel=json.loads((ROOT/'build/mecanum_navigation/kernel_tests.json').read_text())
    legacy=json.loads((ROOT/'build/mecanum/tests/result.json').read_text())
    gate=json.loads((ROOT/'build/mecanum_navigation/grid_gate_result.json').read_text())
    vfh_gate=json.loads((ROOT/'build/mecanum_navigation/vfh_synthesis/gate_result.json').read_text())
    histogram_gate=json.loads((ROOT/'build/mecanum_navigation/histogram_synthesis/gate_result.json').read_text())
    perception=json.loads((ROOT/'build/tests/perception_result.json').read_text())
    control_equivalence=json.loads((ROOT/'build/mecanum_navigation/control_equivalence.json').read_text())
    hardware_path=folder/'result.json'
    hardware=json.loads(hardware_path.read_text()) if hardware_path.exists() else None
    routed_path=folder/'routing_diagnosis.json'
    routed_resources=json.loads(routed_path.read_text()).get('trial',{}).get('resources',{}) if routed_path.exists() else {}
    nav_timing_path=folder/'navigation_timing_audit.json'
    nav_timing=json.loads(nav_timing_path.read_text()) if nav_timing_path.exists() else None
    bit=folder/'impl/pnr/sensor.fs';reference=folder/'impl/pnr/sensor.pr'
    build_log=(folder/'build.log').read_text(errors='replace') if (folder/'build.log').exists() else ''
    complete_outputs='Bitstream generation completed' in build_log and '[100%] Power analysis completed' in build_log
    candidate_ready=bool(hardware and complete_outputs and bit.is_file() and reference.is_file() and reference.stat().st_size>0 and
                         (hardware.get('actual_exit_code')==0 or hardware.get('cli_terminated_after_completed_outputs')) and
                         hardware.get('violations')==dict(setup=0,hold=0) and
                         all(hardware.get('system_audits',{}).get(name) is True for name in ('control50','lk_state','cdc_timing','frame_read_setup')) and
                         hardware.get('min_pulse_width_slack_ns') is not None and
                         nav_timing and nav_timing['passed'])
    full_sta=bool(candidate_ready and hardware.get('passed'))
    assert synthesis['synthesis_gate_passed'] and integration['passed'] and kernel['passed'] and legacy['passed']
    assert integration['source_sha256']==manifest['source_sha256']
    assert all(current_source_hash(p,manifest)==h==sha(folder/'source'/p) for p,h in manifest['source_sha256'].items())
    assert all(sha(ROOT/p)==h for p,h in kernel['source_sha256'].items())
    assert all(sha(ROOT/'tests/navigation'/(name+'.sv'))==result['testbench_sha256'] for name,result in kernel['tests'].items())
    assert sha(ROOT/'tools/mecanum/navigation_model.py')==kernel['model_sha256']
    assert gate['passed'] and all(sha(ROOT/p)==h for p,h in gate['source_sha256'].items())
    assert vfh_gate['passed'] and sha(ROOT/'fpga/navigation/nav_vfh.sv')==vfh_gate['source_sha256']
    assert histogram_gate['passed'] and all(sha(ROOT/p)==h for p,h in histogram_gate['source_sha256'].items())
    assert perception['passed'] and all(sha(ROOT/p)==h for p,h in perception['source_sha256'].items())
    assert control_equivalence['passed'] and all(sha(ROOT/p)==h for p,h in control_equivalence['source_sha256'].items())
    for name in ('lk_math_synth.json','flow_recovery_synth.json'):
        result=json.loads((ROOT/'build/tests'/name).read_text())
        assert result['passed'] and result['netlist_sha256']==synthesis['netlist_sha256'],name
    evidence=ROOT/'docs/evidence/mecanum_navigation_20261103'
    if manifest.get('incremental_pnr',{}).get('explicit_full_rebuild'):evidence=evidence/('implementation_'+folder.name)
    evidence.mkdir(parents=True,exist_ok=True)
    files=[]
    for filename in ('manifest.json','navigation_synthesis_audit.json','system_test_result.json','control50_audit.json','lk_state_audit.json','incremental_rejection.json'):
        source=folder/filename
        if source.exists():shutil.copy2(source,evidence/filename);files.append(source)
    for filename in ('result.json','navigation_timing_audit.json','timing_audit.json','frame_read_timing_audit.json','routing_diagnosis.json','stage_observations.jsonl','service_resource_counts.json','functional_reuse.json'):
        source=folder/filename
        if source.exists():shutil.copy2(source,evidence/filename)
    for name,path in [('kernel_tests.json',ROOT/'build/mecanum_navigation/kernel_tests.json'),
                      ('legacy_mecanum_tests.json',ROOT/'build/mecanum/tests/result.json'),
                      ('legacy_navigation_tests.json',ROOT/'build/task4/validation.json'),
                      ('rollback.json',ROOT/'build/mecanum_navigation/delivery/rollback.json')]:
        shutil.copy2(path,evidence/name)
    shutil.copy2(ROOT/'build/mecanum_navigation/grid_gate_result.json',evidence/'grid_gate_result.json')
    shutil.copy2(ROOT/'build/mecanum_navigation/grid_synthesis/simulation_gate.log',evidence/'grid_primitive_simulation.log')
    for name,path in [('vfh_gate_result.json',ROOT/'build/mecanum_navigation/vfh_synthesis/gate_result.json'),
                      ('vfh_primitive_simulation.log',ROOT/'build/mecanum_navigation/vfh_synthesis/gate.log'),
                      ('histogram_gate_result.json',ROOT/'build/mecanum_navigation/histogram_synthesis/gate_result.json'),
                      ('histogram_primitive_simulation.log',ROOT/'build/mecanum_navigation/histogram_synthesis/simulation.log'),
                      ('perception_result.json',ROOT/'build/tests/perception_result.json'),
                      ('control_equivalence.json',ROOT/'build/mecanum_navigation/control_equivalence.json'),
                      ('lk_math_synth.json',ROOT/'build/tests/lk_math_synth.json'),
                      ('flow_recovery_synth.json',ROOT/'build/tests/flow_recovery_synth.json'),
                      ('service_synthesis_comparison.json',ROOT/'build/mecanum_navigation/service_synthesis/comparison.json')]:
        shutil.copy2(path,evidence/name)
    for name in ('synthesis.log','build.log','simulation.log','compile.log'):
        if (folder/name).exists():shutil.copy2(folder/name,evidence/name)
    shutil.copy2(folder/'impl/gwsynthesis/sensor_syn.rpt.html',evidence/'sensor_syn.rpt.html')
    for path in (ROOT/'build/mecanum_navigation').glob('python_*.log'):
        shutil.copy2(path,evidence/path.name)
    for path in (ROOT/'build/mecanum_navigation').glob('sta_fix_*.log'):
        shutil.copy2(path,evidence/path.name)
    delivery=ROOT/'build/mecanum_navigation/delivery';delivery.mkdir(parents=True,exist_ok=True)
    candidate=None
    if candidate_ready:
        candidate=delivery/('mecanum_navigation_'+folder.name+'.fs')
        # Gowin marks the source bitstream read-only; copy2 preserves it.
        # Repackaging the same immutable build must verify, not overwrite it.
        if candidate.exists():assert sha(candidate)==sha(bit), 'Existing candidate changed'
        else:shutil.copy2(bit,candidate)
        assert sha(candidate)==sha(bit)==hardware['bitstream_sha256']
        (delivery/'candidate.json').write_text(json.dumps(dict(path=str(candidate),sha256=sha(candidate),
            complete_pr_reference=str(reference),complete_pr_sha256=sha(reference),setup_hold=hardware['violations'],
            min_pulse_width_slack_ns=hardware['min_pulse_width_slack_ns'],clock_route_warnings=hardware['clock_route_warnings'],
            full_sta_signoff=full_sta,experimental_boundary=not full_sta,calibration_required=True,programmed=False,flash_written=False),indent=2)+'\n')
        shutil.copy2(delivery/'candidate.json',evidence/'candidate.json')
    pending=['physical calibration and vehicle acceptance'] if candidate_ready else list(synthesis['pending'])
    if candidate_ready and not full_sta:pending.insert(0,'recorded clock routing / minimum pulse width experimental boundary; full STA signoff is false')
    release=dict(functional_tests_passed=True,synthesis_gate_passed=True,hardware_release=False,
                 candidate_ready=candidate_ready,full_sta_signoff=full_sta,
                 new_navigation_sram_candidate=str(candidate) if candidate else None,flash_written=False,programmed=False,
                 frozen_build=str(folder),kernel_benches=len(kernel['tests']),mecanum_benches=len(legacy['benches']),
                 integrated_raw_points=integration['raw_points_checked'],synthesis=synthesis['usage'],
                 routed_resources=routed_resources,
                 blocked_by=synthesis['implementation_blocker'],pending=pending)
    (evidence/'release.json').write_text(json.dumps(release,indent=2)+'\n')
    report = '# 麦轮导航验证与实现门槛\n\n'
    report += '冻结整机：`'+folder.name+'`。功能仿真及综合通过；未做实车验收或下载。\n\n'
    if integration.get('verification_reused_from'):
        report += '本轮仅改变实现选项；RTL/CST/SDC逐字节相同，功能证据复用自`'+Path(integration['verification_reused_from']).name+'`；物理实现与时序审核全部重跑，详见functional_reuse.json。\n\n'
    report += '| 项目 | 结果 |\n|---|---|\n'
    report += '| 导航模块 | '+str(len(kernel['tests']))+'组RTL bench通过；含100个独立SAT/密集扫掠对照、角度环绕、FIFO溢出、CDC停钟/复位与代次门控 |\n'
    report += '| 旧麦轮兼容性 | '+str(len(legacy['benches']))+'组RTL bench通过；保留原任务2旋转验收 |\n'
    report += '| 整机集成 | 12000个原始雷达点、25圈、视频、M4/P4/N4遥测CRC、未使能GPIO通过；没有实车运动验证 |\n'
    for label,value in synthesis['usage'].items():
        report += '| 综合 '+label+' | '+value+' |\n'
    report += '| 地图 / 相机缓存 | 地图4 SDPB；相机缓存129 SDPB保持不变 |\n'
    report += '| RAM实际原语时序 | 当前地图单元综合后SDPB功能模型回归通过，READ_MODE=0；不是整机门级仿真或路由时序签核 |\n'
    report += '| DSP宏 | '+', '.join(k+'='+v for k,v in synthesis['dsp_macros_not_routed_block_count'].items())+'；未换算为已路由物理块签核 |\n'
    if candidate_ready:
        report += '| 实现 / Setup / Hold | 全量整机实现完成；0 / 0违例；导航CDC '+str(nav_timing['covered_capture_registers'])+'个动态捕获位物理路径通过 |\n'
        report += '| 实际布局后资源 | '+', '.join(k+'='+v for k,v in routed_resources.items())+' |\n'
        report += '| 最小脉宽 / 时钟警告 | '+str(hardware['min_pulse_width_slack_ns'])+'ns；PR1014等警告 '+str(len(hardware['clock_route_warnings']))+'条；完整STA签核='+str(full_sta)+' |\n'
        report += '| 新导航SRAM候选 / Flash | 已生成并核验SHA256；未下载、未写Flash；仍须实测标定与车辆验收 |\n\n'
        report += '本轮全量整机实现来自用户明确授权，后续默认增量策略不变。完整.pr、物理时序/扇出/拥塞及CDC报告随冻结工程保留。\n\n'
    else:
        report += '| 时序 / CDC物理约束 / 拥塞 | 实现审核尚未全部通过；不发布新导航候选 |\n'
        report += '| 新导航SRAM候选 / Flash | 未发布新导航位流；未下载、未写Flash |\n\n'
    report += '旧回退SRAM仅含原整机，保留PR1014与MPW -0.849ns边界，不能标成完整STA通过。\n\n'
    report += '证据：本目录 release.json、manifest.json、navigation_synthesis_audit.json、system_test_result.json、kernel_tests.json、sensor_syn.rpt.html、build.log（或synthesis.log）；逐拍设计和复核见上级docs的MECANUM_NAV文档。\n'
    (evidence/'README.md').write_text(report,encoding='utf-8')
    archive=delivery/'mecanum_navigation_reviewed_source.zip'
    extra=list((ROOT/'docs').glob('MECANUM_NAV*'))+[ROOT/'config/mecanum.navigation.pending.json',ROOT/'fpga/mecanum/local_overrides.json',ROOT/'AGENTS.md',ROOT/'docs/WHOLE_SYSTEM_INCREMENTAL.md']
    extra+=list((ROOT/'tests/navigation').glob('tb_mc_nav_*.sv'))+[ROOT/'tests/mecanum/tb_nav_owner.sv',ROOT/'tests/test_navigation.py',ROOT/'tests/mecanum/test_web.py']
    extra+=list((ROOT/'tests/navigation/reference').glob('mc_nav_system_pre_serial.*'))
    extra+=list((ROOT/'tests/navigation/reference').glob('nav_math_full_reference.*'))
    extra+=list((ROOT/'tests/navigation/reference').glob('nav_histogram_reference.*'))
    extra+=list((ROOT/'host/mecanum').glob('*.py'))+[ROOT/'host/mecanum/web.html']
    extra+=list((ROOT/'shared').glob('*.py'))
    extra+=[ROOT/'host/mecanum/motor.json',ROOT/'hardware/mecanum.json',ROOT/'hardware/pins.csv']
    extra+=list((ROOT/'vendor/notices').rglob('*'))
    extra+=list((ROOT/'tools/mecanum').glob('*.py'))+[ROOT/'tools/task4/build_ethernet.py',ROOT/'tools/task4/incremental.py']
    extra+=[ROOT/'tools/task4'/p for p in ('test.py','test_math.py','test_vfh.py','test_display.py','test_ethernet.py')]
    extra+=[ROOT/'tools/test_perception.py',ROOT/'tools/test_lk_math_synth.py',ROOT/'tools/test_flow_recovery_synth.py',
            ROOT/'tests/vision/tb_flow_pipeline.v',ROOT/'tests/navigation/tb_histogram.sv',ROOT/'tests/navigation/tb_vfh.sv',
            ROOT/'tests/mecanum/tb_pwm_width_equivalence.sv',ROOT/'tests/mecanum/reference/mc_at8236_pwm_before_width.sv',
            ROOT/'tests/vision/tb_event_mux_equivalence.v',ROOT/'tests/navigation/reference/event_packet_mux_reference.v']
    extra+=list(evidence.glob('*'))
    extra+=list((ROOT/'build/mecanum_navigation').glob('tb_mc_nav_*.log'))+[ROOT/'build/mecanum_navigation/sweep_vectors.txt',ROOT/'output/playwright/navigation-final.png']
    extra+=list((ROOT/'build/mecanum_navigation').glob('python_*.log'))
    extra+=[ROOT/'build/mecanum_navigation/grid_synthesis/impl/gwsynthesis/grid.vg',ROOT/'build/mecanum_navigation/grid_synthesis/impl/gwsynthesis/grid_syn.rpt.html',ROOT/'build/mecanum_navigation/grid_synthesis/syn.tcl']
    extra+=[ROOT/'build/mecanum_navigation/vfh_synthesis'/p for p in ('tb_vfh_gate.sv','gate_output.txt','syn.tcl','impl/gwsynthesis/vfh.vg','impl/gwsynthesis/vfh_syn.rpt.html')]
    extra+=[ROOT/'build/mecanum_navigation/histogram_synthesis'/p for p in ('wrapper.sv','bench.sv','syn.tcl','impl/gwsynthesis/hist.vg')]
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr('READ_THIS_FIRST.txt',
                   'Reviewed navigation source overlay for the existing fpga-system-integration repository.\n'
                   'Not a standalone full repository. Existing vendor tests/recorded fixtures remain prerequisites.\n'
                   'Root RTL keeps safe default NAVIGATION=0. frozen_project has explicit NAVIGATION=1/LIDAR_RAW=1.\n'
                   'Frozen build.tcl reproduces the explicitly authorized full build; synthesis_only.tcl, if present, runs synthesis only.\n'
                   'No .pr or new .fs is included in this source overlay; candidate.json records separate output paths and hashes.\n'
                   'This navigation full implementation was explicitly authorized. Later tasks still default to whole-system incremental PnR.\n'
                   'No SRAM programming or Flash writing has been performed.\n'
                   'Separate rollback/ directory contains only the OLD pre-navigation SRAM file with its recorded timing limitations.\n')
        for relative in manifest['source_sha256']:
            z.write(ROOT/relative,relative)
            z.write(folder/'source'/relative,'frozen_project/source/'+relative)
        for relative in ('manifest.json','sensor.gprj','sensor.cst','sensor.sdc','build.tcl','synthesis_only.tcl','incremental_rejection.json','impl/gwsynthesis/sensor_syn.rpt.html'):
            if (folder/relative).exists():z.write(folder/relative,'frozen_project/'+relative)
        for path in extra:
            if path.is_file():z.write(path,path.relative_to(ROOT).as_posix())
    (delivery/'archive.json').write_text(json.dumps(dict(path=str(archive),sha256=sha(archive),new_bitstream_included=False,rollback_manifest=str(delivery/'rollback.json')),indent=2)+'\n')
    print(json.dumps(release,indent=2))
    print(str(archive))


if __name__=='__main__':
    main()
