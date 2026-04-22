"""Small subprocess manager for operator-station launchers."""

from __future__ import annotations

import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class LaunchSpec:
    name: str
    command: Sequence[str]
    cwd: str | None = None
    env: dict[str, str] | None = None


@dataclass
class _RunningProcess:
    spec: LaunchSpec
    process: subprocess.Popen = field(repr=False)


class ProcessManager:
    def __init__(self) -> None:
        self._processes: list[_RunningProcess] = []

    def start(self, spec: LaunchSpec) -> subprocess.Popen:
        proc = subprocess.Popen(
            list(spec.command),
            cwd=spec.cwd,
            env=spec.env,
        )
        self._processes.append(_RunningProcess(spec=spec, process=proc))
        return proc

    def running(self) -> list[_RunningProcess]:
        return [entry for entry in self._processes if entry.process.poll() is None]

    def wait_until_any_exit(self, poll_interval_sec: float = 0.2) -> tuple[str, int]:
        while True:
            for entry in self._processes:
                code = entry.process.poll()
                if code is not None:
                    return entry.spec.name, code
            time.sleep(poll_interval_sec)

    def terminate_all(self, sigint_timeout_sec: float = 3.0, terminate_timeout_sec: float = 2.0) -> None:
        running = self.running()
        if not running:
            return

        for entry in running:
            entry.process.send_signal(signal.SIGINT)

        self._wait_for_exit(sigint_timeout_sec)

        running = self.running()
        for entry in running:
            entry.process.terminate()

        self._wait_for_exit(terminate_timeout_sec)

        running = self.running()
        for entry in running:
            entry.process.kill()

        self._wait_for_exit(1.0)

    def _wait_for_exit(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self.running():
                return
            time.sleep(0.1)
