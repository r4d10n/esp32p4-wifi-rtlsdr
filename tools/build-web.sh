#!/usr/bin/env bash
# Build the Vue/TS frontend into components/websdr/dist/, then build the ESP-IDF
# project. `idf.py` will embed the dist assets via the websdr component.
set -euo pipefail

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
repo="$(cd -- "${here}/.." &>/dev/null && pwd)"

cd "${repo}/web"
if [ ! -d node_modules ]; then
  npm install
fi
npm run build

cd "${repo}"
if [ -z "${IDF_PATH:-}" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/esp/v5.5.1/esp-idf/export.sh"
fi
idf.py build "$@"
