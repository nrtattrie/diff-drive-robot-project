
# Claude Global Instructions — INSTALL TO ~/.claude/CLAUDE.md

# Copy this file to ~/.claude/CLAUDE.md on each machine (Mac, Pi, etc.)

# Do NOT edit this copy directly — edit ~/.claude/CLAUDE.md on your primary machine, then sync back.

## Behavior Rules

- In all interactions and commit messages, be extremely concise and sacrifice grammar for the sake of concision.
- Work one phase at a time. Stop and confirm before advancing phases.
- Do not make file edits without explicit approval.
- On session start, read @docs/phases.md and confirm current phase.

## Project State

- See @docs/phases.md for current phase and progress. If this file does not exist, create it.
- See @docs/architecture.md for technical decisions. If this file does not exist, create it.
- See @docs/session-*.md for session states before clearing context. The * will be populated by a date wtih format "YYYY-$

## Phase Execution Rules

When working through docs/phases.md:

- Complete only ONE unchecked item at a time
- After completing it, check the box, then STOP
- Report what was done and wait for explicit user approval
- Do not proceed to the next item until the user says so
- Never check off an item you have not fully completed and verified

## GitHub

- Your primary method for interacting with GitHub should be the GitHub CLI.

## Git

- When creating branches, prefix with nate/ to indicate they came from me.

## Plans

- At the end of each plan, give me a list of unresolved questions to answer, if any. Make the questions extremely concise. Sacrifice grammar for the sake of concision.

## Session Management

Before clearing context, always run /save-session to preserve state.
Session files live in docs/sessions/.
On session start, check docs/sessions/ for the most recent file.
If the session file already exists at the matching date, append the new information to the file.
