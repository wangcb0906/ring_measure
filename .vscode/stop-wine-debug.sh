#!/bin/bash

PID_FILE="/tmp/ring_cali-winedbg.pid"
SOCAT_PID_FILE="/tmp/ring_cali-socat.pid"

if [[ -f "$SOCAT_PID_FILE" ]]; then
    kill "$(cat "$SOCAT_PID_FILE")" 2>/dev/null || true
    rm -f "$SOCAT_PID_FILE"
fi

if [[ -f "$PID_FILE" ]]; then
    kill "$(cat "$PID_FILE")" 2>/dev/null || true
    rm -f "$PID_FILE"
fi

echo "Wine debugger stopped."