# New Project Manual Checklist

## GitHub repository settings

- Set repository visibility (private/public).
- Enable branch protection for `main`.
- Enable secret scanning and push protection.
- Configure CODEOWNERS/review requirement if needed.

## Local environment

- Install required runtimes for selected profile.
- Configure AI CLI auth locally (`.env`, keychain, or provider login).
- Verify scripts run from repository root.

## Project-specific decisions

- Confirm scope and acceptance criteria in `docs/seminar/01-task.md`.
- Choose source citation format and update references workflow.
- Decide review depth and gate criteria for PR merges.

## Requirement traceability

- Run `scripts/helpers/check-requirements.sh` on the target machine.
- Record missing tools in `STATE.md` before starting work.
- Keep `config/requirements.list` aligned with the active profile and toolchain.
