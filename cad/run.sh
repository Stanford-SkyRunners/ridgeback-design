#!/usr/bin/env bash
# Run a Ridgeback (SR-2) CAD script with the shared venv. ridgeback/cad and ridgeback/cad/parts come
# first on the path, so `import design` gets SR-2's design.py (not SR-1's in club/cad).
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONPATH="$HERE:$HERE/parts"
exec $SKY_BUILD/cadenv/bin/python "$@"
