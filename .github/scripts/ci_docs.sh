#!/usr/bin/env bash
# Runs `cargo doc`, honoring CI_HOST_TARGET and an optional CI_DOCS_FEATURES override.
set -euo pipefail

host_target_args() {
  if [ -n "${CI_HOST_TARGET:-}" ]; then
    echo "--target" "${CI_HOST_TARGET}"
  fi
}

if [ -n "${CI_DOCS_FEATURES:-}" ]; then
  # shellcheck disable=SC2046,SC2086
  cargo doc --lib --no-deps ${CI_DOCS_FEATURES} $(host_target_args)
else
  # shellcheck disable=SC2046,SC2086
  cargo doc ${DOCS_ARGS:---lib --no-deps --all-features} $(host_target_args)
fi
