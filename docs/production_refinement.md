# Production Refinement Workflow

Use this workflow when improving the repository piece by piece without changing current behavior.

## Gate

Run the offline gate before and after every small edit:

```bash
make production-gate
```

The gate currently checks:

- Python syntax for repository-owned scripts.
- Unit/contract tests in `tests/`.
- Shell syntax for deployment scripts.
- JavaScript syntax for the dashboard files.

The default gate is offline. It must not require robot access and must not publish robot commands.

After the offline gate passes, run the robot service check only when the robot PC is reachable:

```bash
python3 scripts/production_gate.py --live
```

## Refinement Loop

1. Start from a clean or understood git status.
2. Run `make production-gate` and treat the result as the baseline.
3. Pick one narrow slice, such as one function, one script, one UI panel, or one deployment file.
4. Make the smallest production-quality improvement that preserves behavior.
5. Run `make production-gate` again.
6. Review `git diff` and commit only that slice.

## Robot-Safety Rules

- Do not make command endpoints easier to trigger.
- Keep `armed=true` and `i_understand_risk=true` checks in front of robot command execution.
- Keep live robot checks separate from offline checks.
- Add or update contract tests before changing command, DDS, systemd, or teleoperation behavior.
- Prefer characterization tests first when current behavior is unclear.

## Good First Slices

- Split pure formatting/conversion helpers out of `server.py` and cover them with tests.
- Add tests for `lowstate_to_dict`, `handstate_to_dict`, and command payload validation.
- Update README safety language so it matches the current command-capable dashboard.
- Reduce duplicated deployment patching logic after tests cover the generated file changes.
