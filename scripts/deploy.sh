#!/usr/bin/env bash
# Copy field_control source to a new, inert Pi release directory.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: deploy.sh --host HOST [--user USER] [--identity PATH] [--dry-run]

Copies this repository to a newly created release beneath the fixed remote path
~/apps/field_control/releases. It never changes an existing release or runtime
path. This script never starts field_control, manages services, installs
packages, or passes hardware/CAN/arming options.
EOF
}

host=""
user="johannes"
identity="${HOME}/.ssh/field_control_deploy_ed25519"
dry_run=false

while (($#)); do
    case "$1" in
        --host) host="${2:-}"; shift 2 ;;
        --user) user="${2:-}"; shift 2 ;;
        --identity) identity="${2:-}"; shift 2 ;;
        --dry-run) dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        --target) echo "--target is not supported; releases use the fixed safe path" >&2; exit 2 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ -n "$host" ]] || { echo "--host is required" >&2; exit 2; }
[[ -n "$user" ]] || { echo "--user must not be empty" >&2; exit 2; }
[[ -f "$identity" && -r "$identity" ]] || {
    echo "--identity must name an existing readable regular private-key file" >&2
    exit 2
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(cd -- "$script_dir/.." && pwd)"
remote="$user@$host"
release_base="apps/field_control/releases"
release_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"
release_path="$release_base/$release_id"
ssh_opts=(-i "$identity" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
printf -v identity_quoted '%q' "$identity"
rsync_ssh="ssh -i $identity_quoted -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3"

echo "Preflight: checking $remote for a documented field_control runtime or web listener..."
# Reject only documented runtime invocations so that SSH/Codex command text
# cannot cause a false positive: either ``python -m field_control.cli`` or the
# installed ``field-control`` console script (with or without its venv path).
# Also reject a live web listener. A failed remote inspection remains
# fail-closed.
if ssh "${ssh_opts[@]}" "$remote" 'processes="$(ps -eo args=)" || exit $?; printf "%s\n" "$processes" | grep -Eq "(^|[[:space:]])(-m[[:space:]]field_control[.]cli|([^[:space:]]*/)?field-control)([[:space:]]|$)"; match_rc=$?; case "$match_rc" in 0) exit 10 ;; 1) ;; *) exit "$match_rc" ;; esac; listeners="$(ss -ltnH "sport = :8080" 2>&1)"; ss_rc=$?; [ "$ss_rc" -eq 0 ] || exit "$ss_rc"; if [ -n "$listeners" ]; then printf "%s\n" "$listeners" | grep -q "^LISTEN[[:space:]]"; listener_rc=$?; case "$listener_rc" in 0) exit 11 ;; 1) exit 12 ;; *) exit "$listener_rc" ;; esac; fi; exit 0'; then
    :
else
    rc=$?
    if [[ "$rc" -eq 10 ]]; then
        echo "Refusing deployment: the documented field_control runtime is running on the Pi." >&2
    elif [[ "$rc" -eq 11 ]]; then
        echo "Refusing deployment: TCP port 8080 is already listening on the Pi." >&2
    else
        echo "Refusing deployment: unable to confirm that field_control is stopped (SSH/preflight exit $rc)." >&2
    fi
    exit 1
fi

echo "Planned isolated release: ~/$release_path"
if "$dry_run"; then
    echo "Dry run complete. No remote files, directories, or compile checks were changed/run."
    exit 0
fi

# The release directory itself must be new. This script never deletes, replaces,
# symlinks, or otherwise changes an existing release or runtime path.
echo "Creating new isolated release directory: ~/$release_path"
ssh "${ssh_opts[@]}" "$remote" "mkdir -p -- '$release_base' && mkdir -- '$release_path'"

rsync_opts=(-az --exclude=.git --exclude=.venv --exclude=venv --exclude=__pycache__ --exclude=.DS_Store --exclude=diagnostics)
echo "Copying source to ~/$release_path/src"
rsync -e "$rsync_ssh" "${rsync_opts[@]}" "$source_dir/" "$remote:$release_path/src/"

echo "Running syntax-only compile check on the new deployed source..."
ssh "${ssh_opts[@]}" "$remote" "python3 -m compileall -q '$release_path/src'"
echo "Deployment complete at ~/$release_path. field_control was not started or armed."
