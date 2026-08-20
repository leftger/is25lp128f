#!/usr/bin/env bash
# Runs `cargo check` against the pinned MSRV toolchain.
set -euo pipefail

host_target_args() {
  if [ -n "${CI_HOST_TARGET:-}" ]; then
    echo "--target" "${CI_HOST_TARGET}"
  fi
}

# shellcheck disable=SC2046,SC2086
cargo "+${MSRV}" check ${CHECK_ARGS:---lib --all-features} $(host_target_args)
