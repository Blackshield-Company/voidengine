# Source from the repo root on the box: points the linker and loader at a
# repo-local SDL2 shim (.sdlshim/, untracked) so builds don't depend on /tmp.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LIBRARY_PATH="$here/.sdlshim${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$here/.sdlshim${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
