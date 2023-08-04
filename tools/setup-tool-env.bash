#!/bin/bash

# Original Authors:
#   Sanaa Hamel (trivial, path/filename modifications)

# bail on cmd err (e), unset vars (u), and errors mid-pipe
set -eu
set -o pipefail

if [ "$EUID" -eq 0 ]; then
  echo "[ERROR] This script must not run as root. Exiting."
  exit 1
fi

usage() {
  echo "Usage: $0 [-f]" 1>&2
  exit 1
}

FORCE_UPDATE=0

# Parse command line arguments
while getopts "fh" arg; do
  case $arg in
  f) FORCE_UPDATE=1 ;;
  h) usage ;;
  *) usage ;;
  esac
done

# Find ROOT_DIR from the pathname of this script
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
VENV_COMMITS_RECORD="$VENV_DIR/nevermore_dependency_commits"

PYTHON_REQUIREMENTS=(
  "$ROOT_DIR/requirements.txt"
  "$ROOT_DIR/../klipper/requirements.txt"
)

PYTHON_GITHUB_REPOS=(
  "sanaahamel/serial-flash-py"
)

if [ -f "$VENV_COMMITS_RECORD" ]; then
  TIMESTAMP_LAST_UPDATED="$(date -r "$VENV_COMMITS_RECORD" "+%s")"
else
  TIMESTAMP_LAST_UPDATED=0
fi

file-dependency-is-stale() {
  if [[ "$TIMESTAMP_LAST_UPDATED" -lt "$(date -r "$DEP" "+%s")" ]]; then
    echo "Last update older than \`$DEP\`."
    return 0
  fi

  return 1
}

install-prerequisites() {
  if ! which jq &>/dev/null; then
    echo "installing \`jq\`..."
    sudo apt-get install jq -y
  fi
}

up-to-date() {
  if [[ $FORCE_UPDATE = 1 ]]; then
    echo "Forced updated specified (\`-f\`)."
    return 1
  fi

  if [ ! -f "$VENV_COMMITS_RECORD" ]; then
    echo "No commits record file found. Assuming out of date."
    return 2
  fi

  for COMMIT in "$@"; do
    if ! grep -q -F "$COMMIT" "$VENV_COMMITS_RECORD"; then
      echo "Commit record missing \`$COMMIT\`"
      return 3
    fi
  done

  local FILE_DEPENDENCIES=("${BASH_SOURCE[0]}" "${PYTHON_REQUIREMENTS[@]}")
  for DEP in "${FILE_DEPENDENCIES[@]}"; do
    if file-dependency-is-stale "$DEP"; then
      echo "Dependency is newer: \`$DEP\`"
      return 4
    fi
  done

  return 0
}

error-handler() {
  echo "Something went wrong during the update."
  echo "Would you like to delete the tool environment entirely?"
  echo "This can help if stale dependencies caused the issue,"
  echo "but the next update can take much longer since it's starting from scratch."

  while true; do
    read -r -p "Delete the tool environment cache? [yn]" ANSWER
    case $ANSWER in
    [Yy])
      rm -rf "$VENV_DIR"
      break
      ;;
    [Nn]) break ;;
    *) echo "Delete the tool environment cache? [yn]" ;;
    esac
  done

  exit 1
}

trap error-handler ERR

# fetch before install.
# if we race we fail safe by getting the old commit, but installing the latest.
# idempotent - future update check will notice an 'out of date' and update to the latest
COMMIT_SERIAL_FLASH_PY="$(curl -s -L \
  -H "Accept: application/vnd.github+json" \
  -H "X-GitHub-Api-Version: 2022-11-28" \
  https://api.github.com/repos/sanaahamel/serial-flash-py/branches/main | jq -r .commit.sha)"

if up-to-date "$COMMIT_SERIAL_FLASH_PY"; then
  echo "Tool environment seems up to date."
  exit 0
fi

install-prerequisites

echo "Updating tool environment..."
echo "Be patient. This may take a long time..."
echo ""

rm -rf "$VENV_COMMITS_RECORD"

if [ ! -d "$VENV_DIR" ] && ! virtualenv -p python3 "$VENV_DIR"; then
  echo "[ERR] Failed to create \`${VENV_DIR}\`"
  rm -rf "$VENV_DIR"
  exit 1
fi

for REQ in $PYTHON_REQUIREMENTS; do
  "$VENV_DIR/bin/pip" install -r "$REQ"
done

for REPO in $PYTHON_GITHUB_REPOS; do
  "$VENV_DIR/bin/pip" install "git+https://github.com/$REPO"
done

echo "$COMMIT_SERIAL_FLASH_PY" >"$VENV_COMMITS_RECORD"
