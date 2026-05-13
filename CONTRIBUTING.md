# Contributing to term-pcl

Thank you for helping improve `term-pcl`. Keep contributions focused, tested, and easy to review.

## Development setup

Install build dependencies on Ubuntu/Debian:

```bash
sudo apt update
sudo apt install cmake g++ make libpcl-dev git ca-certificates
```

Configure, build, and test from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Contribution workflow

1. Open an issue before large behavior changes or broad refactors.
2. Keep pull requests focused on one problem or feature.
3. Add or update tests for user-visible behavior changes and bug fixes.
4. Preserve existing supported file formats and normal CLI behavior unless the existing behavior is unsafe.
5. Do not commit generated build directories, package artifacts, local editor state, or large sample data.

## Coding expectations

- Use C++17-compatible code.
- Prefer small, direct changes over broad abstractions.
- Keep CLI errors concise and actionable.
- Validate data at file and CLI boundaries.
- Avoid adding new dependencies unless they are necessary for the feature or fix.

## Packaging expectations

If you change build, install, or Debian packaging files, verify the normal build and consider the package build path:

```bash
dpkg-buildpackage -us -uc -b
```

If package tests are intentionally skipped for a local packaging check, use:

```bash
DEB_BUILD_OPTIONS=nocheck dpkg-buildpackage -us -uc -b
```
