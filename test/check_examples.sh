#!/bin/sh
# Syntax/link-check every example against the mock Arduino environment.
set -e
cd "$(dirname "$0")"
fail=0
for ino in ../examples/*/*.ino; do
  name=$(basename "$ino" .ino)
  tmp=$(mktemp -d)
  { echo '#include <Arduino.h>'; cat "$ino"; echo 'int main(){ setup(); loop(); return 0; }'; } > "$tmp/$name.cpp"
  # the mock needs millis()/Wire definitions: reuse the ones from the test harness
  if g++ -std=gnu++11 -Wall -Wextra -c -Imock -I../src "$tmp/$name.cpp" -o "$tmp/$name.o" 2> "$tmp/err.txt"; then
    echo "OK   $name"
  else
    echo "FAIL $name"; cat "$tmp/err.txt"; fail=1
  fi
  rm -rf "$tmp"
done
exit $fail
