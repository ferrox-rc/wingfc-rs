## Description
<!-- Provide a brief description of the changes introduced in this PR. -->

## Type of Change
- [ ] 🚀 New Feature (non-breaking change adding functionality)
- [ ] 🐛 Bug Fix (non-breaking change fixing an issue)
- [ ] ⚡ Performance Optimization
- [ ] 📝 Documentation Update
- [ ] 🔧 Refactoring / Code Quality Improvement
- [ ] ⚠️ Breaking Change

## Flight Controller Pre-Flight Checklist
- [ ] `./scripts/build.sh test` passes all host unit tests across feature matrix.
- [ ] `./scripts/build.sh check` validates `wingfc.yaml` configuration.
- [ ] `./scripts/build.sh matrix check` succeeds for all 8 target firmware permutations.
- [ ] `cargo fmt --all -- --check` reports zero formatting discrepancies.
- [ ] No hardcoded local machine paths (e.g. `file:///home/...`) exist in documentation.
- [ ] Documentation updated in `docs/` or `README.md` if applicable.

## Testing & Verification
<!-- Describe the tests or hardware simulations conducted to verify these changes. -->
