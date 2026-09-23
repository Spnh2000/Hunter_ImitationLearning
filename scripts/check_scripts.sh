#!/bin/bash
# Syntax-check every script here. Committed a pause script with an unterminated
# string on 2026-09-23; `bash -n` would have caught it in 50 ms.
set -e
cd "$(dirname "$0")"
fail=0
for f in *.sh; do
  bash -n "$f" || { echo "SYNTAX ERROR: $f"; fail=1; }
done
[ $fail -eq 0 ] && echo "all scripts parse"
exit $fail
