"""Run the YOLO agent and mixed tracking/flow observation endpoint."""
from pathlib import Path
import runpy,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'vendor/vision'))
sys.path.insert(0,str(ROOT/'nano/vision'))
runpy.run_path(str(ROOT/'nano/vision/vision_dashboard_agent.py'),run_name='__main__')
