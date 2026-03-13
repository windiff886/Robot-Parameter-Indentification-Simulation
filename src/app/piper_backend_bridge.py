#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_real.bridge import run_stdio_loop


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Piper 真机后端桥接进程")
    parser.add_argument(
        "--config",
        default=str(REPO_ROOT / "config" / "piper_real_experiment.yaml"),
        help="真机实验配置文件路径",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return run_stdio_loop(Path(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
