# FSDS_DOCKER - DEPRECATED

**WARNING: This directory is DEPRECATED. Use `src/autonomous/` for all development.**

## STATUS

Legacy monolithic structure. Kept for reference only.

## WHERE TO GO

| If you're looking for... | Go to |
|--------------------------|-------|
| Main driver | `../src/autonomous/driver/` |
| Control algorithms | `../src/autonomous/modules/control/` |
| Perception code | `../src/autonomous/modules/perception/` |
| Configuration | `../src/autonomous/config/params.yaml` |
| Tests | `../src/autonomous/tests/` |

## DO NOT

- Add new code here
- Modify existing code
- Import from this directory
- Reference in new documentation

## STRUCTURE (historical)

```
fsds_docker/
├── scripts/     # Monolithic legacy scripts (DO NOT USE)
├── src/         # Partially modularized (DO NOT USE)
└── tests/       # Legacy tests (DO NOT USE)
```

## MIGRATION

All functionality has been migrated to `src/autonomous/`. This directory will be removed in a future cleanup.
