#!/usr/bin/env sh
set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
output=${1:-"$root/artifacts/validation"}
build="$output/build"
sanitizers=${ASR_FC_ENABLE_SANITIZERS:-ON}

run_logged() {
    log=$1
    shift
    if "$@" >"$log" 2>&1; then
        cat "$log"
    else
        status=$?
        cat "$log"
        return "$status"
    fi
}

rm -rf "$output"
mkdir -p "$output"

"$root/scripts/capture_runner_evidence.sh" "$output/runner-hardware.txt"
git -C "$root" rev-parse HEAD >"$output/git-commit.txt"
git -C "$root" submodule status --recursive >"$output/submodules.txt"
"${CC:-cc}" --version >"$output/compiler.txt" 2>&1

run_logged "$output/configure.log" cmake -S "$root" -B "$build" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DASR_FC_ENABLE_SANITIZERS="$sanitizers" \
    -DCMAKE_C_FLAGS='-Wall -Wextra -Wpedantic -Werror'
run_logged "$output/build.log" cmake --build "$build" --parallel
run_logged "$output/ctest.log" ctest --test-dir "$build" --output-on-failure \
    --output-junit "$output/ctest.xml"

{
    printf '# ASR-FC validation\n\n'
    printf -- '- Commit: `%s`\n' "$(cat "$output/git-commit.txt")"
    printf -- '- Compiler: `%s`\n' "$(head -n 1 "$output/compiler.txt")"
    printf -- '- Sanitizers: `%s`\n' "$sanitizers"
    printf -- '- Result: native configure, build, and tests passed\n'
} >"$output/summary.md"

printf 'ASR-FC validation artifacts: %s\n' "$output"
