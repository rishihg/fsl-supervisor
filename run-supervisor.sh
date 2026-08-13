#!/bin/bash
# Starts hst_supervisor.py inside a tmux session named "hst" (or attaches to
# it if it's already running), so the session can always be reattached from
# another machine with: tmux attach -t hst
#
# Do NOT run `python3 hst_supervisor.py` directly — a copy started outside
# tmux holds the single-instance lock but has no tmux session to attach to,
# leaving anyone who ssh's in unable to reach it. Use this script instead.
#
# Run as `./run-supervisor.sh` (this directory isn't on PATH by default), or
# symlink it onto PATH once per account, e.g.:
#     ln -s /home/qshanty/supervisor/run-supervisor.sh ~/bin/run-supervisor.sh
# so it works as a bare command from any directory over SSH.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION=hst
LOCK_FILE=/tmp/hst_supervisor.lock

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' is already running, attaching..."
    exec tmux attach -t "$SESSION"
fi

# No tmux session — check whether the single-instance lock is nonetheless
# held, which means a stray copy is running outside tmux (e.g. started
# directly in a terminal on the local desktop).
if ! ( exec 9<>"$LOCK_FILE"; flock -n 9 ); then
    lock_pid="$(cat "$LOCK_FILE" 2>/dev/null || true)"
    echo "============================================================"
    echo "ERROR: hst_supervisor is already running (lock held) but there"
    echo "is no tmux session named '$SESSION' to attach to."
    echo
    if [ -n "$lock_pid" ]; then
        tty="$(ps -o tty= -p "$lock_pid" 2>/dev/null | tr -d ' ')"
        echo "It's running as PID $lock_pid${tty:+ on tty $tty}."
        echo "That's likely a copy started directly (not via this script)"
        echo "in a terminal on the machine's local desktop session."
    else
        echo "Could not determine the PID holding the lock."
    fi
    echo
    echo "Find that terminal and quit the app cleanly from there (so"
    echo "hardware shuts down safely), then re-run this script."
    echo "============================================================"
    exit 1
fi

echo "Starting hst_supervisor in new tmux session '$SESSION'..."
tmux new-session -d -s "$SESSION" "cd '$SCRIPT_DIR' && exec python3 hst_supervisor.py"
exec tmux attach -t "$SESSION"
