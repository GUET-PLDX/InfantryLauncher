#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
test_dir=$(mktemp -d /tmp/heat_rate_ctrl_test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT

# Test the production heat model without the embedded runtime dependencies.
{
  printf '#include <algorithm>\n#include <cmath>\n'
  sed -n '/^namespace launcher {/,/^}  \/\/ namespace launcher/p' \
    "$script_dir/../InfantryLauncher.hpp"
} > "$test_dir/heat_rate_ctrl.inc"

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -I"$test_dir" \
  "$script_dir/heat_rate_ctrl_test.cpp" -o "$test_dir/heat_rate_ctrl_test"
"$test_dir/heat_rate_ctrl_test"
printf 'PASS: InfantryLauncher heat rate control regression\n'
