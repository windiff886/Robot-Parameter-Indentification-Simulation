#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_real.config import load_real_experiment_config
from piper_real.runner import RealExperimentRunner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="运行 Piper 真机实验")
    parser.add_argument(
        "--config",
        default=str(REPO_ROOT / "config" / "piper_real_experiment.yaml"),
        help="真机实验配置文件路径",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_real_experiment_config(Path(args.config), repo_root=REPO_ROOT)
    runner = RealExperimentRunner(config)
    runner.run()
    print(f"真机实验完成，数据已保存到: {config.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
