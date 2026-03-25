#!/usr/bin/env bash
# Serve documentation locally
# Usage: ./docs/serve.sh [port]

PORT="${1:-8000}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Documentation server starting..."
echo "  Python docs: http://localhost:$PORT"
echo "  C++ docs:    http://localhost:$((PORT+1))"
echo ""
echo "Press Ctrl+C to stop"

# Serve Python docs on main port, C++ on port+1
python3 -m http.server "$PORT" -d "$SCRIPT_DIR/_build/html" &
PID1=$!
python3 -m http.server "$((PORT+1))" -d "$SCRIPT_DIR/_build/doxygen/html" &
PID2=$!

trap "kill $PID1 $PID2 2>/dev/null" EXIT
wait
