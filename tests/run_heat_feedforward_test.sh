#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
test_dir=$(mktemp -d /tmp/heat_feedforward_test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT

# Test the production heat model without the embedded runtime dependencies.
{
  printf '#include <algorithm>\n#include <cmath>\n'
  sed -n '/^namespace launcher {/,/^}  \/\/ namespace launcher/p' \
    "$script_dir/../InfantryLauncher.hpp"
} > "$test_dir/heat_feedforward.inc"

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -I"$test_dir" \
  "$script_dir/heat_feedforward_test.cpp" -o "$test_dir/heat_feedforward_test"
"$test_dir/heat_feedforward_test"
printf 'PASS: InfantryLauncher heat feedforward regression\n'
