"""Keep the fault-injection regressions in normal test discovery."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from audit_host_logic import HostLogicAudit
