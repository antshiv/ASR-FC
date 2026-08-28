#!/usr/bin/env sh
set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
output=${1:-"$root/artifacts/validation"}
build="$output/build"
sanitizers=${ASR_FC_ENABLE_SANITIZERS:-ON}

rm -rf "$output"
mkdir -p "$output"

"$root/scripts/capture_runner_evidence.sh" "$output/runner-hardware.txt"
git -C "$root" rev-parse HEAD >"$output/git-commit.txt"
git -C "$root" submodule status --recursive >"$output/submodules.txt"
"${CC:-cc}" --version >"$output/compiler.txt" 2>&1

cmake -S "$root" -B "$build" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DASR_FC_ENABLE_SANITIZERS="$sanitizers" \
    -DCMAKE_C_FLAGS='-Wall -Wextra -Wpedantic -Werror' \
    2>&1 | tee "$output/configure.log"
cmake --build "$build" --parallel 2>&1 | tee "$output/build.log"
ctest --test-dir "$build" --output-on-failure \
    --output-junit "$output/ctest.xml" 2>&1 | tee "$output/ctest.log"

{
    printf '# ASR-FC validation\n\n'
    printf -- '- Commit: `%s`\n' "$(cat "$output/git-commit.txt")"
    printf -- '- Compiler: `%s`\n' "$(head -n 1 "$output/compiler.txt")"
    printf -- '- Sanitizers: `%s`\n' "$sanitizers"
    printf -- '- Result: native configure, build, and tests passed\n'
} >"$output/summary.md"

printf 'ASR-FC validation artifacts: %s\n' "$output"
