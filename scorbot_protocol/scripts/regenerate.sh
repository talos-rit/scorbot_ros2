#!/usr/bin/env bash
# Regenerate the committed nanopb code from proto/scorbot/v1/scorbot.proto.
#
# Requires the nanopb generator at the same version as the vendored runtime
# (third_party/nanopb/pb.h, NANOPB_VERSION). Install once with:
#   python3 -m pip install "nanopb==0.4.9.1" grpcio-tools
#
# CI runs this and fails if the output differs from what is committed, so the
# generated files can never drift from the .proto.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(dirname "$here")"
proto_dir="$pkg/proto"
out_dir="$pkg/generated"

expected="$(grep -o 'nanopb-[0-9.]*' "$pkg/third_party/nanopb/pb.h" | head -1)"
actual="$(python3 -c 'import importlib.metadata as m; print("nanopb-" + m.version("nanopb"))')"
if [ "$expected" != "$actual" ]; then
  echo "nanopb generator $actual does not match vendored runtime $expected" >&2
  echo "install with: python3 -m pip install \"${expected#nanopb-}\" grpcio-tools" >&2
  exit 1
fi

mkdir -p "$out_dir"
python3 -m nanopb.generator.nanopb_generator \
  -I "$proto_dir" \
  -D "$out_dir" \
  --strip-path \
  "$proto_dir/scorbot/v1/scorbot.proto"

echo "generated:"
find "$out_dir" -type f | sort
