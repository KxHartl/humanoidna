# AGENTS.md

Global operating rules for humans and AI agents in this repository.

## Roles

- **Planner**: decomposes goals into tasks and updates `STATE.md` backlog.
- **Researcher**: collects and summarizes sources in `docs/references/`.
- **Coder**: implements code in `src/` and analysis scripts in `analysis/`.
- **Analyst**: transforms data from `data/raw` to `data/processed`.
- **Writer**: produces seminar/thesis text in `docs/seminar/`.
- **Reviewer**: reviews code/text quality and writes reports into `reviews/`.

## Non-negotiable rules

1. Never commit directly to `main`.
2. One task = one branch + one worktree.
3. Keep commits scoped and descriptive.
4. Preserve source-of-truth data: never edit files in `data/raw/`.
5. Cite sources in `docs/references/` whenever claims are added to seminar text.
6. Reviewer must only use task brief + diff + standards; no implementer chat history.

## Branch and worktree naming

- Branches:
  - `task/<slug>`
  - `review/<task-slug>-<agent>`
  - `docs/<slug>`
  - `fix/<slug>`
- Worktrees:
  - `.agents/<slug>`

## Expected task flow

1. Create task worktree with `scripts/git/new-task-worktree.sh`.
2. Execute work in the worktree.
3. Run quality checks relevant to profile.
4. Create review report in `reviews/code/` or `reviews/text/`.
5. Open PR and merge after review.
6. Remove worktree with `scripts/git/cleanup-worktrees.sh`.
