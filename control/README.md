# AgentRealm (GitHub Template)

Universal GitHub template for **projects, seminars, and thesis** work using a shared human + AI-agent workflow.

> [!WARNING]
> **This is a TEMPLATE (a factory mold), not a live project repo.**
> Do not work in this repository directly. Follow the instructions below to start a new project.

---

## 🚀 How to Start a New Project

1. **GitHub Template**: Go to the [AgentRealm](https://github.com/your-username/agentRealm) repository on GitHub.
2. **Use this template**: Click the green **"Use this template"** button.
   - ❌ **DO NOT FORK**: Forking is for contributing to *this* template. "Use this template" is for starting a *new* project.
3. **Private Repository**: Create a new private repository under your account.
4. **Clone locally**: Clone your new project repository to your machine.
5. **Bootstrap**: Run the setup script to initialize your profile (e.g., python, document).
   ```bash
   ./scripts/helpers/bootstrap-project.sh --name "My New Project" --profile python
   ```

---

## 🛡️ The Golden Rules for AI-Agent Collaboration

To prevent chaos and keep your repository clean, follow these rules:

1. **1 Agent = 1 Task = 1 Git Worktree**: Each AI agent or task must work in its own isolated sandbox.
2. **Never Commit to `main` Directly**: Always work in branches. `main` is the "source of truth."
3. **Review Before Merging**: Use Pull Requests (PRs) or manual diff reviews (`git diff`) before merging any agent's work.
4. **No Secrets**: Never put API keys or secrets in the template files. Use `.env` files (which are git-ignored).

---

## 🛠️ Daily Workflow (Idiot-Proof)

### Step 1: Create a Task Sandbox
Use the script to create a new git worktree (sandbox) for your task.
```bash
./scripts/git/new-task-worktree.sh literature-review
```
This creates a folder in `.agents/literature-review` and checks out a new branch.

### Step 2: Run an Agent
Start your preferred agent (Claude, Gemini, Copilot, etc.) inside that worktree.
```bash
./scripts/agents/run_copilot_task.sh .agents/literature-review
```

### Step 3: Review the Work
Go to the worktree folder, check the changes, and commit them.
```bash
cd .agents/literature-review
git status
git diff
git add .
git commit -m "feat: completed literature review outline"
```

### Step 4: Merge and Cleanup
Push the branch, open a PR on GitHub, merge it to `main`, and then delete the sandbox.
```bash
# From the main repo folder
./scripts/git/cleanup-worktrees.sh .agents/literature-review
```

---

## 📁 Folder Structure Explained

- `docs/`: Your writing (seminar/thesis), references, and [templates](./docs/TEMPLATES.md).
- `src/`: Your implementation code.
- `analysis/`: Data notebooks and reports.
- `data/`: Raw and processed data (never edit `data/raw/`).
- `assets/`: Figures, diagrams, and style guides.
- `config/`: Agent roles and project configuration.
- `scripts/`: Help scripts for git and agent management.
- `.agents/`: Local git worktree sandboxes (ignored by main git history until merged).

---

## 🤖 Global Agent State

- `AGENTS.md`: The rulebook. Every agent reads this to know their role and constraints.
- `STATE.md`: The live brain. This file tracks what has been done, what is being worked on, and the backlog.
