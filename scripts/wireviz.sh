#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
wireviz_bin="${project_root}/venv/wireviz/bin/wireviz"

if [[ ! -x "${wireviz_bin}" ]]; then
  echo "WireViz is not installed at ${wireviz_bin}" >&2
  exit 1
fi

# ROS may export Python paths that should not leak into this isolated tool.
exec env -u PYTHONHOME -u PYTHONPATH "${wireviz_bin}" "$@"
