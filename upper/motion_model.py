"""Simulation uses the production STM32 allocator, compiled for the PC."""
import ctypes
from pathlib import Path
import subprocess
import sys
import threading


_load_lock=threading.Lock()
_loaded_function=None


def _allocator_sources(root):
    source=root/'embedding/f4/firmware'
    paths=[root/'upper/tools/motion_model.cpp',root/'upper/tools/build_pc_motion_model.py',
           source/'config/messages/BodyCommand.h',source/'config/messages/ActuatorFrame.h',
           source/'config/messages/MotionTypes.h',source/'APP/Algorithm/GaitPhase.h',
           source/'APP/Algorithm/PairedLimit.h',
           source/'Device/Actuator/EscCalibration.h']
    for folder in ('Mode/Motion','Task','Device/Actuator','config'):
        paths.extend(path for path in (source/folder).rglob('*')
                     if path.suffix in ('.h','.cpp'))
    return [path for path in paths if path.is_file()]


def _needs_rebuild(library,sources):
    try:
        built_at=library.stat().st_mtime_ns
    except FileNotFoundError:
        return True
    return any(path.stat().st_mtime_ns>built_at for path in sources)


def allocator():
    # Check only before the first load in this process. A running workstation
    # keeps its loaded DLL; updated sources are picked up on its next startup.
    global _loaded_function
    with _load_lock:
        if _loaded_function is not None:
            return _loaded_function
        library_path=Path(__file__).with_name('motion_model.dll')
        root=Path(__file__).resolve().parents[1]
        if _needs_rebuild(library_path,_allocator_sources(root)):
            subprocess.run([sys.executable,'-B','-X','utf8',
                            str(root/'upper/tools/build_pc_motion_model.py')],cwd=root,check=True)
        library=ctypes.CDLL(str(library_path))
        function=library.rf_pc_allocate_brake
        function.argtypes=[ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,
                           ctypes.c_float,ctypes.c_uint,ctypes.POINTER(ctypes.c_float)]
        function.restype=ctypes.c_int
        _loaded_function=function
        return function


def allocate(mode,axes,now_ms):
    values=(ctypes.c_float*6)()
    if not allocator()(mode,axes['surge'],axes['yaw'],axes['heave'],axes.get('brake',0),int(now_ms)&0xffffffff,values):
        raise ValueError('无效三轴目标')
    return dict(zip(('tail1','tail2','left_fin','right_fin','left_esc','right_esc'),values))
