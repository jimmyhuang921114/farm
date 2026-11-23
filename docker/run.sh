#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image="${FARM_IMAGE:-farm-test:humble}"
container="${FARM_CONTAINER:-farm-test}"
serial_device="${FARM_SERIAL_DEVICE-}"

if ! docker container inspect --format '{{.State.Running}}' "$container" 2>/dev/null | grep -qx true; then
    if ! docker image inspect "$image" >/dev/null 2>&1; then
        printf 'Image %s is missing. Build it with %s/docker/build.sh\n' "$image" "$workspace" >&2
        exit 1
    fi

    device_args=()
    if [[ -n "$serial_device" ]]; then
        if [[ ! -c "$serial_device" ]]; then
            printf 'Serial device is unavailable: %s\n' "$serial_device" >&2
            exit 1
        fi
        device_args=(--device "$serial_device:/dev/ttyACM0"
                     --group-add "$(stat -Lc '%g' -- "$serial_device")")
    fi

    docker run --detach --rm --init --name "$container" \
        --network host --ipc host \
        --user "$(id -u):$(id -g)" --env HOME=/tmp \
        --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
        --mount "type=bind,source=$workspace,target=/farm" \
        "${device_args[@]}" "$image" sleep infinity >/dev/null
fi

if (( $# == 0 )); then
    set -- bash
fi

exec_args=(--interactive)
if [[ -t 0 && -t 1 ]]; then
    exec_args+=(--tty)
fi

exec docker exec "${exec_args[@]}" "$container" /ros_entrypoint.sh "$@"
