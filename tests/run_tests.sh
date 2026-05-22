#!/usr/bin/env bash
#
# run_tests.sh — drive the Walkie SDK hardware test scripts in a sensible order.
#
# Suites (climb the ladder; don't advance until the current one is green):
#   offline : test_connection.py --offline-only          (no robot needed)
#   safe    : connection, telemetry, cameras, visualization   (zero motion)
#   motion  : lift, navigation, arm                       (⚠️ MOVES HARDWARE)
#   all     : safe + motion
#
# Motion scripts prompt [y/N] before each move unless you pass --yes.
#
# Usage:
#   tests/run_tests.sh --ip 192.168.1.100                 # safe suite (default)
#   tests/run_tests.sh --ip 192.168.1.100 --all
#   tests/run_tests.sh --ip 192.168.1.100 --motion --yes
#   tests/run_tests.sh --offline                          # no robot
#   tests/run_tests.sh --ip 192.168.1.100 --namespace robot1 --camera-extra "--multi --show"
#   tests/run_tests.sh --ip 192.168.1.100 --safe --stop-on-fail
#
set -u

# ── Defaults ─────────────────────────────────────────────────────────────────
IP="127.0.0.1"
PORT="9090"
NAMESPACE=""
SUITE="safe"            # offline | safe | motion | all
YES=""                  # "--yes" to skip motion prompts
STOP_ON_FAIL=0
CAMERA_EXTRA=""         # e.g. "--multi --show"
ARM_MODE="moveit"       # moveit | custom_ik | both

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN=(uv run python)

usage() {
    # Print the contiguous comment header (skip shebang, stop at first code line).
    awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
}

# ── Arg parsing ──────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip)            IP="$2"; shift 2;;
        --port)          PORT="$2"; shift 2;;
        --namespace)     NAMESPACE="$2"; shift 2;;
        --offline)       SUITE="offline"; shift;;
        --safe)          SUITE="safe"; shift;;
        --motion)        SUITE="motion"; shift;;
        --all)           SUITE="all"; shift;;
        --yes|-y)        YES="--yes"; shift;;
        --stop-on-fail)  STOP_ON_FAIL=1; shift;;
        --camera-extra)  CAMERA_EXTRA="$2"; shift 2;;
        --arm-mode)      ARM_MODE="$2"; shift 2;;
        -h|--help)       usage; exit 0;;
        *) echo "Unknown argument: $1" >&2; usage; exit 2;;
    esac
done

cd "$REPO_ROOT" || exit 1

# Namespace flag (omitted entirely when empty; test_connection.py doesn't accept it)
NS_FLAG=()
[[ -n "$NAMESPACE" ]] && NS_FLAG=(--namespace "$NAMESPACE")

# ── Result tracking ──────────────────────────────────────────────────────────
declare -a NAMES=()
declare -a CODES=()

run_test() {
    local name="$1"; shift
    echo
    echo "########################################################################"
    echo "#  RUN: $name"
    echo "#  cmd: $*"
    echo "########################################################################"
    "$@"
    local code=$?
    NAMES+=("$name")
    CODES+=("$code")
    if [[ $code -ne 0 ]]; then
        echo ">> '$name' exited non-zero ($code)."
        if [[ $STOP_ON_FAIL -eq 1 ]]; then
            echo ">> --stop-on-fail set; halting."
            summary
            exit "$code"
        fi
    fi
}

confirm_motion() {
    if [[ -n "$YES" ]]; then
        return 0
    fi
    echo
    echo "⚠️  About to run MOTION tests (lift / navigation / arm) on ${IP}."
    echo "    Keep the workspace clear and an e-stop within reach."
    read -r -p "    Continue? [y/N] " ans
    [[ "$ans" == "y" || "$ans" == "Y" || "$ans" == "yes" ]]
}

# ── Suites ───────────────────────────────────────────────────────────────────
run_offline() {
    run_test "connection(offline)" "${RUN[@]}" "$SCRIPT_DIR/test_connection.py" --offline-only
}

run_safe() {
    run_test "connection" "${RUN[@]}" "$SCRIPT_DIR/test_connection.py" --ip "$IP" --port "$PORT"
    run_test "telemetry"  "${RUN[@]}" "$SCRIPT_DIR/test_telemetry.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}"
    # shellcheck disable=SC2086
    run_test "cameras"    "${RUN[@]}" "$SCRIPT_DIR/test_multi_camera.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}" $CAMERA_EXTRA
    run_test "visualization" "${RUN[@]}" "$SCRIPT_DIR/test_visualization.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}"
}

run_motion() {
    if ! confirm_motion; then
        echo ">> Motion tests skipped by user."
        return
    fi
    run_test "lift"       "${RUN[@]}" "$SCRIPT_DIR/test_lift.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}"
    # shellcheck disable=SC2086
    run_test "navigation" "${RUN[@]}" "$SCRIPT_DIR/test_navigation.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}" $YES
    # shellcheck disable=SC2086
    run_test "arm"        "${RUN[@]}" "$SCRIPT_DIR/test_arm.py" --ip "$IP" --port "$PORT" "${NS_FLAG[@]}" --mode "$ARM_MODE" $YES
}

# ── Summary ──────────────────────────────────────────────────────────────────
summary() {
    echo
    echo "========================================================================"
    echo "  SUMMARY  (suite=$SUITE, ip=$IP:$PORT)"
    echo "========================================================================"
    local fails=0
    for i in "${!NAMES[@]}"; do
        if [[ "${CODES[$i]}" -eq 0 ]]; then
            printf "  [PASS] %s\n" "${NAMES[$i]}"
        else
            printf "  [FAIL] %s (exit %s)\n" "${NAMES[$i]}" "${CODES[$i]}"
            fails=$((fails + 1))
        fi
    done
    echo "------------------------------------------------------------------------"
    printf "  %d/%d scripts passed\n" "$(( ${#NAMES[@]} - fails ))" "${#NAMES[@]}"
    echo "========================================================================"
    return "$fails"
}

# ── Main ─────────────────────────────────────────────────────────────────────
echo "Walkie SDK test runner — suite='$SUITE', target=${IP}:${PORT}"
case "$SUITE" in
    offline) run_offline;;
    safe)    run_safe;;
    motion)  run_motion;;
    all)     run_safe; run_motion;;
    *) echo "Unknown suite: $SUITE" >&2; exit 2;;
esac

summary
exit $?
