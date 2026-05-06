# Agent Implementation Brief

## Objective

Set up this repository for a specific project using the selected profile.

## Steps

1. Run:
   ```bash
   ./scripts/helpers/bootstrap-project.sh --name "<PROJECT_NAME>" --profile <python|cpp|document>
   ```
2. Update:
   - `config/project.yaml`
   - `STATE.md` (current focus + backlog)
   - `docs/seminar/01-task.md` (project brief)
3. Create first task worktree:
   ```bash
   ./scripts/git/new-task-worktree.sh setup-initial
   ```
4. Execute task in `.agents/setup-initial` with selected AI CLI wrapper.
5. Write review output in `reviews/code/` or `reviews/text/`.
6. Open PR and merge.
