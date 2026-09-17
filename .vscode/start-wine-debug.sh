#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEPLOY_DIR="$PROJECT_ROOT/build-win/deploy"

FIXED_PORT=40780

LOG_FILE="/tmp/ring_cali-winedbg.log"
SOCAT_LOG="/tmp/ring_cali-socat.log"

PID_FILE="/tmp/ring_cali-winedbg.pid"
SOCAT_PID_FILE="/tmp/ring_cali-socat.pid"

cd "$DEPLOY_DIR"

# Clean up old processes/files
if [[ -f "$PID_FILE" ]]; then
    kill "$(cat "$PID_FILE")" 2>/dev/null || true
    rm -f "$PID_FILE"
fi

if [[ -f "$SOCAT_PID_FILE" ]]; then
    kill "$(cat "$SOCAT_PID_FILE")" 2>/dev/null || true
    rm -f "$SOCAT_PID_FILE"
fi

rm -f "$LOG_FILE" "$SOCAT_LOG"

echo "Starting Wine debugger..."

stdbuf -oL -eL winedbg --gdb --no-start ./ring_cali.exe \
    cali/20260622 \
    -o auto \
    -p auto.ini \
    -m 1 \
    >"$LOG_FILE" 2>&1 &

WINEGDB_PID=$!
echo "$WINEGDB_PID" > "$PID_FILE"

echo "WineDbg PID: $WINEGDB_PID"

# Wait for WineDbg to report its GDB port
WINE_PORT=""

for i in {1..300}; do

    if [[ -f "$LOG_FILE" ]]; then
        WINE_PORT=$(sed -nE \
            's/.*target remote localhost:([0-9]+).*/\1/p' \
            "$LOG_FILE" | head -n1)
    fi

    if [[ -n "$WINE_PORT" ]]; then
        break
    fi

    if ! kill -0 "$WINEGDB_PID" 2>/dev/null; then
        echo "ERROR: WineDbg exited unexpectedly."
        cat "$LOG_FILE"
        exit 1
    fi

    sleep 0.1
done

if [[ -z "$WINE_PORT" ]]; then
    echo "ERROR: Could not determine Wine GDB port."
    cat "$LOG_FILE"
    exit 1
fi

echo "Wine GDB port: $WINE_PORT"

# Start socat
socat \
    "TCP-LISTEN:${FIXED_PORT},reuseaddr,fork" \
    "TCP:127.0.0.1:${WINE_PORT}" \
    >"$SOCAT_LOG" 2>&1 &

SOCAT_PID=$!
echo "$SOCAT_PID" > "$SOCAT_PID_FILE"

# Wait until socat is listening
for i in {1..100}; do

    if ss -lnt | grep -q ":${FIXED_PORT} "; then
        echo "GDB proxy ready on localhost:${FIXED_PORT}"
        break
    fi

    if ! kill -0 "$SOCAT_PID" 2>/dev/null; then
        echo "ERROR: socat exited unexpectedly."
        cat "$SOCAT_LOG"
        exit 1
    fi

    sleep 0.1
done

# Make sure the proxy really became ready
if ! ss -lnt | grep -q ":${FIXED_PORT} "; then
    echo "ERROR: socat did not open port ${FIXED_PORT}"
    cat "$SOCAT_LOG"
    exit 1
fi

echo "GDB proxy ready on localhost:${FIXED_PORT}"

# Keep the VS Code background task alive
wait "$WINEGDB_PID"