#!/bin/bash
set -e
cd "$(dirname "$0")"
make clean && make all
echo ""
echo "=== ALL TESTS COMPLETE ==="
