"""Private token entry and catchable process termination; standard library only."""
from contextlib import contextmanager
import getpass
import signal
import sys
import threading
import warnings


def private_token():
    if not sys.stdin.isatty():
        raise ValueError('Hardware sessions require a terminal for private TFT token entry')
    with warnings.catch_warnings():
        warnings.simplefilter('error', getpass.GetPassWarning)
        try:
            token = getpass.getpass('Five-character TFT token (hidden): ')
        except getpass.GetPassWarning as error:
            raise ValueError('Private token entry unavailable') from error
    if len(token) != 5 or any(not 33 <= ord(c) <= 126 for c in token):
        raise ValueError('Use five printable ASCII characters shown on the TFT')
    return token


@contextmanager
def termination_cleanup():
    """Unwind finally blocks on SIGTERM/Ctrl+C; repeated signals allow cleanup."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    signals = (signal.SIGINT, signal.SIGTERM)
    previous = {s: signal.getsignal(s) for s in signals}
    def interrupt(signum, frame):
        for s in signals:
            signal.signal(s, signal.SIG_IGN)
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)
    try:
        for s in signals:
            signal.signal(s, interrupt)
        yield
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)


@contextmanager
def finish_cleanup():
    """Do not interrupt an already-running, bounded network cleanup."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        for s in previous:
            signal.signal(s, signal.SIG_IGN)
        yield
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)
