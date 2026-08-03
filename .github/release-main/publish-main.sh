#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <dev-commit> <release-tree>" >&2
  exit 2
fi

source_commit=$1
release_tree=$2
git_dir=$(git rev-parse --absolute-git-dir)
index_dir=$(mktemp -d)
index_file="$index_dir/index"

cleanup() {
  if [[ -e "$index_file" ]]; then
    unlink "$index_file"
  fi
  rmdir "$index_dir" 2>/dev/null || true
}
trap cleanup EXIT

git cat-file -e "${source_commit}^{commit}"

GIT_DIR="$git_dir" \
GIT_WORK_TREE="$release_tree" \
GIT_INDEX_FILE="$index_file" \
  git add --force --all

release_tree_sha=$(
  GIT_DIR="$git_dir" GIT_INDEX_FILE="$index_file" git write-tree
)

git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"

for attempt in 1 2 3; do
  git fetch --no-tags origin \
    '+refs/heads/main:refs/remotes/origin/main' \
    '+refs/heads/dev:refs/remotes/origin/dev'

  current_main=$(git rev-parse refs/remotes/origin/main)
  current_dev=$(git rev-parse refs/remotes/origin/dev)

  if ! git merge-base --is-ancestor "$source_commit" "$current_dev"; then
    echo "source dev commit is no longer reachable from origin/dev; skipping stale deployment"
    exit 0
  fi

  if git merge-base --is-ancestor "$source_commit" "$current_main"; then
    echo "dev commit $source_commit is already represented by main $current_main"
    exit 0
  fi

  release_commit=$(
    git commit-tree "$release_tree_sha" \
      -p "$current_main" \
      -p "$source_commit" \
      -m "release: deploy dev ${source_commit:0:12}" \
      -m "Source-Dev-SHA: $source_commit" \
      -m "Release-Tree-SHA: $release_tree_sha"
  )

  if git push origin "$release_commit:refs/heads/main"; then
    remote_main=$(git ls-remote --heads origin main | cut -f1)
    if [[ "$remote_main" != "$release_commit" ]]; then
      echo "remote main verification failed" >&2
      exit 1
    fi
    echo "published main $release_commit from dev $source_commit"
    exit 0
  fi

  echo "main advanced during publish attempt $attempt; retrying" >&2
done

echo "could not publish main after three race-safe attempts" >&2
exit 1
