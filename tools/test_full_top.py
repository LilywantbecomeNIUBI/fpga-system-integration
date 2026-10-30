"""Run only the actual system-top digital simulation after an RTL change."""
from pathlib import Path
import hashlib,json,os,sys
from build_system import prepare
from test_system import run,ROOT
def main():
    folder,files=prepare();sources=[str(p) for p in files if p.suffix in ('.v','.sv') and p.name!='ethernet_pll.v']
    prim=str(Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/simlib/gw5a/prim_sim.v')
    run('full_compile',['iverilog','-g2012','-s','tb_full_system','-s','GSR','-o','build/tests/full_system.vvp','tests/tb_full_system.sv','tests/imu/icm42688_model.sv','tests/ethernet_pll_model.v',prim]+sources)
    result=run('full_top',['vvp','build/tests/full_system.vvp'],timeout=600)
    sys.path.insert(0,str(ROOT));from shared.streams import MixedStream
    parser=MixedStream();rows=parser.feed((ROOT/'build/tests/full_system_uart.bin').read_bytes())
    assert parser.bad_crc==0 and {'motor','imu','encoder_diagnostics','encoder_trace'}<={kind for kind,row in rows}
    result.update(decoded_records=len(rows),bad_crc=parser.bad_crc,hardware_tested=False,
        telemetry_decimation_fixture=4,telemetry_decimation_production=20,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files+[ROOT/'tests/tb_full_system.sv']})
    (ROOT/'build/tests/full_top_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('PASS full-top mixed UART',len(rows))
if __name__=='__main__':main()
