#!/usr/bin/env bash
set -euo pipefail

binary="$1"
tmpdir="$(mktemp -d "${TMPDIR:-/tmp}/term-pcl-cli.XXXXXX")"
trap 'rm -rf "$tmpdir"' EXIT

help_output="$($binary --help)"
[[ "$help_output" == *"Usage: term-pcl"* ]]
[[ "$help_output" == *".pcd, .ply, .xyz, .xyzn, .xyzrgb"* ]]
[[ "$help_output" == *"--color MODE"* ]]
[[ "$help_output" == *"--force"* ]]

if "$binary" >"$tmpdir/missing.out" 2>"$tmpdir/missing.err"; then
  echo "missing argument should fail" >&2
  exit 1
fi
grep -q "Usage: term-pcl" "$tmpdir/missing.err"

check_output="$($binary --check tests/fixtures/sample.xyz)"
[[ "$check_output" == "Loaded 3 points from sample.xyz" ]]

color_output="$($binary --check --color rgb tests/fixtures/sample.xyz)"
[[ "$color_output" == "Loaded 3 points from sample.xyz" ]]

budget_output="$($binary --check --point-budget 2 tests/fixtures/sample.xyz)"
[[ "$budget_output" == "Loaded 2 points from sample.xyz" ]]

voxel_output="$($binary --check --voxel-size 0.5 tests/fixtures/sample.xyz)"
[[ "$voxel_output" == Loaded*"points from sample.xyz" ]]

profile_output="$($binary --check --profile tests/fixtures/sample.xyz 2>$tmpdir/profile.err)"
[[ "$profile_output" == "Loaded 3 points from sample.xyz" ]]
grep -q "load_ms=" $tmpdir/profile.err

if "$binary" --check --point-budget nope tests/fixtures/sample.xyz >$tmpdir/bad-budget.out 2>$tmpdir/bad-budget.err; then
  echo "invalid point budget should fail" >&2
  exit 1
fi
grep -q "Invalid --point-budget" $tmpdir/bad-budget.err

if "$binary" --check --color nope tests/fixtures/sample.xyz >$tmpdir/bad-color.out 2>$tmpdir/bad-color.err; then
  echo "invalid color mode should fail" >&2
  exit 1
fi
grep -q "Invalid --color" $tmpdir/bad-color.err

output_dir="$tmpdir/sample.termcloud"
index_output="$($binary index tests/fixtures/sample.xyz --output "$output_dir" --point-budget 2)"
[[ "$index_output" == "Indexed 2 points to $output_dir" ]]
[[ -f "$output_dir/metadata.txt" ]]
grep -q "termcloud 3" "$output_dir/metadata.txt"
[[ -f "$output_dir/nodes.txt" ]]
[[ -d "$output_dir/chunks" ]]
indexed_check_output="$($binary --check "$output_dir")"
[[ "$indexed_check_output" == *"Termcloud v3 index: sample.xyz"* ]]
[[ "$indexed_check_output" == *"points=2"* ]]
[[ "$indexed_check_output" == *"nodes="* ]]

if "$binary" index tests/fixtures/sample.xyz --output "$output_dir" >"$tmpdir/existing.out" 2>"$tmpdir/existing.err"; then
  echo "existing output directory should fail without --force" >&2
  exit 1
fi
grep -q "already exists" "$tmpdir/existing.err"
grep -q -- "--force" "$tmpdir/existing.err"

sentinel="$output_dir/stale.txt"
: > "$sentinel"
force_output="$($binary index tests/fixtures/sample.xyz --output "$output_dir" --point-budget 1 --force)"
[[ "$force_output" == "Indexed 1 points to $output_dir" ]]
[[ -f "$output_dir/metadata.txt" ]]
[[ ! -e "$sentinel" ]]

existing_file="$tmpdir/existing-file.termcloud"
: > "$existing_file"
if "$binary" index tests/fixtures/sample.xyz --output "$existing_file" >"$tmpdir/existing-file.out" 2>"$tmpdir/existing-file.err"; then
  echo "existing output file should fail" >&2
  exit 1
fi
grep -q "already exists" "$tmpdir/existing-file.err"

if "$binary" --check tests/fixtures/sample.unsupported >$tmpdir/unsupported.out 2>$tmpdir/unsupported.err; then
  echo "unsupported extension should fail" >&2
  exit 1
fi
grep -q "Unsupported point cloud format" $tmpdir/unsupported.err
