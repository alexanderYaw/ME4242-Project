#!/usr/bin/env bash
# Start the EMIO motor GUI at http://localhost:8000 (Linux / macOS). Ctrl+C to stop.
# Optional: PORT=8001 ./run.sh   EMIO_MOCK=1 ./run.sh (simulated motors)   EMIO_PORT=/dev/ttyUSB1 ./run.sh
set -e
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
if [ ! -f .venv/.installed ]; then
  .venv/bin/pip install -r requirements.txt && touch .venv/.installed
fi
echo "Open http://localhost:${PORT:-8000}"
cd app && exec ../.venv/bin/python -m uvicorn server:app --host 127.0.0.1 --port "${PORT:-8000}"
