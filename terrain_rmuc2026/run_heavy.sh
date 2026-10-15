#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEAVY="/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh"
PYTHON="/home/kaiser/miniconda3/envs/rl-lab/bin/python"
case "${1:-}" in
  cad) shift; exec "$HEAVY" 6144 timeout --signal=TERM 600s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/prepare_cad.py" "$@" ;;
  build) shift; exec "$HEAVY" 6144 timeout --signal=TERM 600s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/build_field.py" "$@" ;;
  preview) shift; exec "$HEAVY" 6144 timeout --signal=TERM 240s env MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/preview.py" "$@" ;;
  check) shift; exec "$HEAVY" 6144 timeout --signal=TERM 600s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/check_field.py" "$@" ;;
  refine-check) shift; exec "$HEAVY" 2048 timeout --signal=TERM 120s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/check_refined.py" "$@" ;;
  view) shift; exec "$HEAVY" 6144 env MUJOCO_GL=glfw OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$PYTHON" -u "$TASK_DIR/view.py" "$@" ;;
  package) shift; exec "$HEAVY" 2048 timeout --signal=TERM 240s "$PYTHON" -u "$TASK_DIR/package_runtime.py" "$@" ;;
  rules) shift; exec "$HEAVY" 1024 timeout --signal=TERM 180s "$PYTHON" -u "$TASK_DIR/download_rules.py" "$@" ;;
  *) echo "Usage: $0 {cad|build|preview|check|refine-check|package|rules|view} [arguments]" >&2; exit 2 ;;
esac
