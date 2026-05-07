# track_plot — agent guide

This repo is a small CARLA-oriented Python script collection. Treat `tp_replay/` as the reusable core package; most other root-level `.py` files are thin CLIs or one-off tools.

## What matters

- `tp_replay/` is the main library boundary: `main.py`, `engine.py`, `models.py`, `config.py`, `env_utils.py`, `world_utils.py`, `geometry.py`, `selection.py`, `carla_compat.py`
- Root scripts are mostly entrypoints or utilities: `auto_control_main.py` (recommended replay entry), `replay_main.py` (compat shim), `trace_display.py`, `trace_plot.py`, `validate_trace_data.py`, `record_spectator_view.py`, `scan_trace_time_ranges.py`
- `trace_data/` contains both data-processing scripts and trace artifacts; `trace_data/stitched/` and `run_logs/` are generated outputs
- Example trace files live at repo root (`data_6lu*.txt`, `data_12lu.txt`)

## Canonical commands

Run from repo root, usually after activating the `carla` conda env:

```powershell
python .\auto_control_main.py
python -m tp_replay.main
python .\trace_plot.py
python .\trace_display.py
python .\record_spectator_view.py
python .\validate_trace_data.py --data-file .\data_6lu2.txt
python -m compileall .
```

CARLA scripts usually assume `localhost:2000`.

## Repo-specific rules

- Prefer `tp_replay/` for new replay logic; keep root scripts as thin wrappers when possible
- Avoid adding new top-level modules unless they are true entrypoints
- Do not hard-code absolute paths; prefer relative paths, env vars, or CLI args
- Keep CARLA usage defensive; use `tp_replay/carla_compat.py` for `carla` import checks when possible
- Preserve existing Chinese comments and section dividers in files that already use them

## Known gotchas

- `README.md` references `test1.py`, but it is not in this repo
- There is no committed test/CI setup and no dependency lockfile (`pyproject.toml`, `requirements.txt`, etc.)
- `README.md` documents the current replay default: preserve the existing CARLA world unless explicitly overridden
- Generated files (`run_logs/`, `trace_data/stitched/`, `__pycache__/`) should not be treated as source of truth

## Editing guidance

- Make minimal, targeted changes
- Prefer small, import-safe helpers over more script-level side effects
- If you change behavior, update the README and keep commands in sync with the real entrypoints
