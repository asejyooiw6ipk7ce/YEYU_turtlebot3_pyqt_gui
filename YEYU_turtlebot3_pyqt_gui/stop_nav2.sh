#!/bin/bash

PID_FILE="$HOME/nav2.pid"

if [ ! -f "$PID_FILE" ]; then
    echo "NOT_RUNNING"
    exit 0
fi

PID=$(cat "$PID_FILE")

if kill -0 "$PID" 2>/dev/null; then
    kill -INT -"$PID" 2>/dev/null
    sleep 2

    if kill -0 "$PID" 2>/dev/null; then
        kill -TERM -"$PID" 2>/dev/null
        sleep 1
    fi

    if kill -0 "$PID" 2>/dev/null; then
        kill -KILL -"$PID" 2>/dev/null
    fi

    rm -f "$PID_FILE"
    echo "STOPPED"
else
    rm -f "$PID_FILE"
    echo "NOT_RUNNING"
fi
