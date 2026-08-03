#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <release-tree>" >&2
  exit 2
fi

release_tree=$1
repo_root=$(git rev-parse --show-toplevel)
release_dir="$repo_root/.github/release-main"
manifest="$release_dir/files.txt"
expected_files=$(mktemp)
actual_files=$(mktemp)
archive_file=$(mktemp --suffix=.tar.gz)
trap 'rm -f "$expected_files" "$actual_files" "$archive_file"' EXIT

LC_ALL=C sort "$manifest" > "$expected_files"
find "$release_tree" -type f -printf '%P\n' | LC_ALL=C sort > "$actual_files"
if ! diff -u "$expected_files" "$actual_files"; then
  echo "generated release does not match the allowlist" >&2
  exit 1
fi

mapfile -t top_level < <(
  find "$release_tree" -mindepth 1 -maxdepth 1 -printf '%f\n' | LC_ALL=C sort
)
expected_top_level=(".gitignore" "README.md" "src")
if [[ "${top_level[*]}" != "${expected_top_level[*]}" ]]; then
  echo "unexpected top-level release contents: ${top_level[*]}" >&2
  exit 1
fi

mapfile -t packages < <(
  find "$release_tree/src" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' |
    LC_ALL=C sort
)
expected_packages=("EPIC_poongsan" "reactive_local_avoidance")
if [[ "${packages[*]}" != "${expected_packages[*]}" ]]; then
  echo "unexpected release packages: ${packages[*]}" >&2
  exit 1
fi

if grep -R -n -E 'ml_x_cropping|livox_ros_driver2' "$release_tree"; then
  echo "forbidden ML-X cropping or Livox dependency remains in release" >&2
  exit 1
fi

config_dir="$release_tree/src/EPIC_poongsan/src/EPIC/src/global_planner/exploration_manager/config"
mapfile -t yaml_profiles < <(
  find "$config_dir" -maxdepth 1 -type f -name '*.yaml' -printf '%f\n' |
    LC_ALL=C sort
)
expected_profiles=("real1.yaml" "real2.yaml")
if [[ "${yaml_profiles[*]}" != "${expected_profiles[*]}" ]]; then
  echo "unexpected runtime profiles: ${yaml_profiles[*]}" >&2
  exit 1
fi

python3 - "$release_tree" <<'PY'
import pathlib
import sys
import xml.etree.ElementTree as ET

root = pathlib.Path(sys.argv[1])
xml_files = sorted(
    path
    for path in root.rglob("*")
    if path.is_file()
    and (
        path.suffix in {".launch", ".xml"}
        or path.name == "package.xml"
    )
)
for path in xml_files:
    ET.parse(path)
print(f"parsed {len(xml_files)} XML/launch files")
PY

release_bytes=$(du -sb "$release_tree" | cut -f1)
max_release_bytes=$((25 * 1024 * 1024))
if (( release_bytes > max_release_bytes )); then
  echo "release tree is too large: $release_bytes bytes" >&2
  exit 1
fi

tar -czf "$archive_file" -C "$release_tree" .
archive_bytes=$(stat -c '%s' "$archive_file")
max_archive_bytes=$((5 * 1024 * 1024))
if (( archive_bytes > max_archive_bytes )); then
  echo "release archive is too large: $archive_bytes bytes" >&2
  exit 1
fi

file_count=$(wc -l < "$actual_files")
echo "release validation passed: $file_count files, $release_bytes bytes, $archive_bytes compressed bytes"
