# Decisions Log

## 2026-04-20
- Odluka: Postavljena `.agent/` arhitektura za multi-model AI asistenciju.
- Zasto:
  - Centraliziran context za CLI alate (Copilot, Gemini, Codex)
  - Minimiziranje tokena kroz modularne `.md` datoteke
  - Git verzioniranje promjena kao audit trail
- Commit pattern: `chore(agent): <kratki opis>` za `.agent/` izmjene
- Kada commitati:
  - nakon svakog zavrsenog koraka
  - nakon svake kriticne promjene
