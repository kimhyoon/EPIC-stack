#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <dev-commit> <new-output-directory>" >&2
  exit 2
fi

source_commit=$1
output_dir=$2
repo_root=$(git rev-parse --show-toplevel)
release_dir="$repo_root/.github/release-main"
manifest="$release_dir/files.txt"
release_patch="$release_dir/main.patch"
release_readme="$release_dir/README.md"
release_gitignore="$release_dir/gitignore.main"

git cat-file -e "${source_commit}^{commit}"

if [[ -e "$output_dir" ]]; then
  echo "output path already exists: $output_dir" >&2
  exit 2
fi

mapfile -t release_files < "$manifest"
if [[ ${#release_files[@]} -eq 0 ]]; then
  echo "release manifest is empty" >&2
  exit 2
fi

for path in "${release_files[@]}"; do
  if [[ -z "$path" || "$path" = /* || "$path" == *".."* ]]; then
    echo "unsafe path in release manifest: $path" >&2
    exit 2
  fi
  if ! git cat-file -e "$source_commit:$path" 2>/dev/null; then
    echo "manifest path is missing from dev commit $source_commit: $path" >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
git archive --format=tar "$source_commit" -- "${release_files[@]}" |
  tar -xf - -C "$output_dir"

git apply --check --unsafe-paths --directory="$output_dir" "$release_patch"
git apply --unsafe-paths --directory="$output_dir" "$release_patch"
install -m 0644 "$release_readme" "$output_dir/README.md"
install -m 0644 "$release_gitignore" "$output_dir/.gitignore"

echo "generated lightweight main tree from dev $source_commit"
