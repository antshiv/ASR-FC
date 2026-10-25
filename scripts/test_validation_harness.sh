#!/usr/bin/env sh
set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT HUP INT TERM
mkdir -p "$temporary/bin"

cat >"$temporary/bin/cmake" <<'EOF'
#!/usr/bin/env sh
if [ "$1" = '--build' ]; then
    stage=build
else
    stage=configure
fi
printf '%s marker\n' "$stage"
if [ "${FAIL_STAGE:-}" = "$stage" ]; then
    exit 23
fi
EOF
cat >"$temporary/bin/ctest" <<'EOF'
#!/usr/bin/env sh
printf 'ctest marker\n'
if [ "${FAIL_STAGE:-}" = test ]; then
    exit 23
fi
EOF
chmod +x "$temporary/bin/cmake" "$temporary/bin/ctest"

for stage in configure build test; do
    output="$temporary/$stage"
    if FAIL_STAGE="$stage" PATH="$temporary/bin:$PATH" \
        "$root/scripts/run_validation.sh" "$output" >"$temporary/stdout" 2>&1; then
        printf 'FAIL: %s failure was swallowed\n' "$stage" >&2
        exit 1
    else
        status=$?
    fi
    if [ "$status" -ne 23 ]; then
        printf 'FAIL: %s returned %s, expected 23\n' "$stage" "$status" >&2
        cat "$temporary/stdout" >&2
        exit 1
    fi
    if [ -f "$output/summary.md" ]; then
        printf 'FAIL: %s failure wrote a passing summary\n' "$stage" >&2
        exit 1
    fi
    case "$stage" in
        configure) log="$output/configure.log" ;;
        build) log="$output/build.log" ;;
        test) log="$output/ctest.log" ;;
    esac
    if ! grep -q "$stage marker" "$log"; then
        printf 'FAIL: %s failure log missing\n' "$stage" >&2
        exit 1
    fi
done

printf 'Validation harness propagates configure, build, and test failures.\n'
