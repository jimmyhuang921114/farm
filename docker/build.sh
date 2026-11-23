#!/usr/bin/env bash
set -euo pipefail

docker_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image="${FARM_IMAGE:-farm-test:humble}"

exec docker build --tag "$image" "$docker_dir" "$@"
