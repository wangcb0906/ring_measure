#!/usr/bin/env bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 <program_command [args...]>"
    echo "Example: $0 ./ring_cali cali/20260622 -o auto"
    exit 1
fi

PROGRAM=("$@")
VALGRIND_LOG="valgrind_report.log"
TEMP_LOG=$(mktemp)

echo "========================================"
echo " [1/2] Running Memory Leak Check (Valgrind)"
echo " Target: ${PROGRAM[*]}"
echo "========================================"

# Run Valgrind with strict error tracking (--error-exitcode=42)
valgrind \
    --leak-check=full \
    --show-leak-kinds=all \
    --track-origins=yes \
    --error-exitcode=42 \
    --log-file="$VALGRIND_LOG" \
    "${PROGRAM[@]}" >/dev/null 2>&1

VALGRIND_EXIT=$?

if [ "$VALGRIND_EXIT" -eq 42 ]; then
    echo "----------------------------------------"
    echo "[!] FAILED: Memory leaks or errors detected!"
    echo "    Detailed report saved to: $(pwd)/$VALGRIND_LOG"
    echo "    Skipping performance profiling until leaks are fixed."
    echo "========================================"
    exit 42
else
    echo "[+] PASSED: No memory leaks detected!"
    rm -f "$VALGRIND_LOG"
fi

echo ""
echo "========================================"
echo " [2/2] Running Performance Profiling"
echo "========================================"

# Run through GNU time verbose (-v) and capture output
/usr/bin/time -v -o "$TEMP_LOG" "${PROGRAM[@]}" >/dev/null 2>&1
EXIT_CODE=$?

echo "----------------------------------------"
echo "PROGRAM FINISHED (Exit Code: $EXIT_CODE)"
echo "========================================"
echo "          FULL PERFORMANCE REPORT       "
echo "========================================"

# Read and format the output from the temp log
while IFS= read -r line; do
    # Check for memory rows in KB and convert them to MB for readability
    if [[ "$line" =~ (size|resident|kbytes) ]]; then
        key=$(echo "$line" | cut -d: -f1)
        val=$(echo "$line" | grep -oE '[0-9]+')
        if [ -n "$val" ]; then
            mb_val=$(awk "BEGIN {print $val/1024}")
            printf "%-35s : %s KB (~%.2f MB)\n" "$key" "$val" "$mb_val"
            continue
        fi
    fi
    # Print all other lines as-is
    echo "$line"
done < "$TEMP_LOG"

echo "========================================"

# Cleanup
rm -f "$TEMP_LOG"