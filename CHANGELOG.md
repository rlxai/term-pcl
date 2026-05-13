# Changelog

All notable changes to `term-pcl` are documented here.

## [Unreleased]

### Added

- Public launch documentation and community files.
- Safer `.termcloud` indexing behavior with explicit overwrite control.
- Regression coverage for CLI indexing, `.termcloud` validation, and chunk loading failures.

### Changed

- `.termcloud --check` now reports index metadata for v3 indexes.
- Existing `.termcloud` outputs are no longer overwritten unless callers pass explicit force options.
- Build configuration no longer forces global optimization flags outside normal CMake build-type handling.

## [0.1.0] - 2026-05-13

### Added

- Native Linux terminal point-cloud viewer.
- Support for `.pcd`, `.ply`, `.xyz`, `.xyzn`, `.xyzrgb`, and `.termcloud` inputs.
- RGB, elevation, x-axis, white, rainbow, turbo, viridis, heat, and grayscale color modes.
- Point-budget loading, voxel downsampling, profiling, hierarchical LOD, lazy chunk streaming, and `.termcloud` indexing.
- Debian package metadata and native Linux CI.
