"""Private, bounded, asynchronous local diagnostics; no UI or console handler."""
import atexit
from datetime import datetime, timezone
import json
import logging
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
from pathlib import Path
import queue
import sys
import threading
import traceback

LOG = logging.getLogger('robotfish.diagnostics')
LOG.addHandler(logging.NullHandler())
LOG.propagate = False
_listener = None
_hooks = None


def active():
    return _listener is not None


class QuietQueueHandler(QueueHandler):
    def enqueue(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            pass  # Disk stalls must never block telemetry or motion commands.


class QuietFileHandler(RotatingFileHandler):
    def handleError(self, record):
        pass


def event(name, **fields):
    if LOG.isEnabledFor(logging.INFO):
        try:
            message = json.dumps(dict(time=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                                      event=name, **fields), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            return
        LOG.info(message)


def start(directory=None):
    global _listener, _hooks
    if _listener is not None:
        return
    directory = Path(directory or Path(__file__).resolve().parent / 'logs')
    directory.mkdir(parents=True, exist_ok=True)
    sink = QuietFileHandler(directory / 'workstation.jsonl', maxBytes=5 * 1024 * 1024,
                            backupCount=3, encoding='utf-8')
    handler = QuietQueueHandler(queue.Queue(maxsize=1024))
    LOG.handlers = [handler]
    LOG.setLevel(logging.INFO)
    _listener = QueueListener(handler.queue, sink)
    _listener.start()
    atexit.register(stop)
    _hooks = sys.excepthook, threading.excepthook
    def crash(kind, trace, **fields):
        event('uncaught_error', error_type=kind.__name__, **fields,
              stack=[dict(file=Path(frame.filename).name, function=frame.name, line=frame.lineno)
                     for frame in traceback.extract_tb(trace)])
    sys.excepthook = lambda kind, value, trace: crash(kind, trace)
    threading.excepthook = lambda args: crash(args.exc_type, args.exc_traceback, thread=args.thread.name)


def stop():
    global _listener, _hooks
    if _listener is not None:
        listener, _listener = _listener, None
        LOG.handlers = [logging.NullHandler()]
        if _hooks is not None:
            sys.excepthook, threading.excepthook = _hooks
            _hooks = None
        listener.queue.join()
        listener.stop()
        for sink in listener.handlers:
            sink.close()
