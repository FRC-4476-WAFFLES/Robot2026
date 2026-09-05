#!/usr/bin/env bash
# Formats staged code before every commit so formatting never depends on anyone
# remembering, or on an editor being configured. Agents write files directly and
# never trigger format-on-save, so this is the only thing that covers them.
#
# Installed automatically by the gradle-pre-commit-git-hooks plugin in
# settings.gradle. Skip it for one commit with `git commit --no-verify`.

DIRTY_BEFORE=$(git diff --name-only)

set -e
./gradlew spotlessApply --quiet
set +e

DIRTY_AFTER=$(git diff --name-only)

# Stage only what Spotless changed. Anything already dirty before this ran is
# work in progress and must not be swept into the commit.
CHANGED=$(comm -13 <(echo "$DIRTY_BEFORE" | sort) <(echo "$DIRTY_AFTER" | sort))

if [ -n "$CHANGED" ]; then
  echo "spotless reformatted:"
  echo "$CHANGED" | sed 's/^/  /'
  echo "$CHANGED" | xargs git add
fi
