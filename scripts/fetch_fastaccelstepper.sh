#!/usr/bin/env bash
set -euo pipefail

# Fetch FastAccelStepper into resources/fastaccelstepper (for offline study)
DESTDIR="$(dirname "$0")/../resources/fastaccelstepper"
REPO="https://github.com/gin66/FastAccelStepper.git"

mkdir -p "$DESTDIR"
if [ -d "$DESTDIR/.git" ]; then
  echo "FastAccelStepper already cloned in $DESTDIR; fetching updates..."
  git -C "$DESTDIR" fetch --all --prune
  git -C "$DESTDIR" pull --ff-only
else
  echo "Cloning FastAccelStepper into $DESTDIR"
  git clone "$REPO" "$DESTDIR"
fi

echo "Done. See $DESTDIR"
