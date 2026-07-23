# CLAUDE.md — instructions for Claude Code in this repository

## Git workflow (REQUIRED — every session)

**Commit and push to git after every single change.** After each edit or set of
edits that leaves the repo in a coherent state, run:

```bash
git add -A
git commit -m "<clear message>"
git push
```

- Work on and push to `main` (the robot's `robot-telemetry-web-autoupdate` timer
  pulls from `origin/main`, so pushing there is how changes reach the robot).
- Do not batch many changes into one commit — commit each change as it is made.
- NEVER add Claude as a contributor: no `Co-Authored-By`, no "Generated with"
  lines, no AI attribution of any kind in commit messages.

## Project

Unitree H1-2 telemetry + XR operator dashboard. Key files: `server.py` (HTTP
server, DDS telemetry, guarded wrist/loco/arm_sdk command endpoints, closed-loop
arm replay), `static/app.js`, `static/viewer.js`, `static/diagram.js` (docs
.drawio viewer). Tests: `python3 -m unittest discover -s tests -p 'test_*.py'`;
release gate: `make production-gate`. This code can move a real robot — keep all
safety interlocks intact and run the tests before pushing.
