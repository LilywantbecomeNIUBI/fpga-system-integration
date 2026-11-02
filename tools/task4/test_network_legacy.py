"""Run the preserved dual-network golden with this repository's source paths."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
original=ROOT/'vendor/vision/test_network_dual_original.py'
source=original.read_text(encoding='utf-8')
source=source.replace('ROOT=Path(__file__).resolve().parents[1]','ROOT=Path('+repr(str(ROOT))+')')
source=source.replace("'sim/tb_network_dual.v'","'tests/vision/tb_network_dual.v'")
source=source.replace("'src/","'fpga/vision/src/")
# The full-system simulation and Gowin may share this workstation. Allow wall
# time for contention; simulation duration and every golden assertion stay exact.
source=source.replace('timeout=45','timeout=180')
exec(compile(source,str(original),'exec'),dict(__file__=str(original),__name__='__main__'))
