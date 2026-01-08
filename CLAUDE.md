# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Python package for converting Lanelet2 map format to OpenDRIVE format, designed for use with Autoware autonomous driving software. The project uses the modern Python packaging tool `uv` for dependency management and builds.

## Development Environment Setup

This project uses `uv` (version 0.9.7+) for Python package management. Ensure uv is installed before working with this codebase.

### Key Commands

```bash
# Install dependencies
uv pip install -e .

# Sync dependencies from lock file
uv sync

# Add a new dependency
uv add <package_name>

# Add a development dependency
uv add --dev <package_name>

# Run Python scripts
uv run python <script.py>

# Create virtual environment (if needed)
uv venv
```

## Project Structure

- `src/autoware_lanelet2_to_opendrive/` - Main Python package directory
  - `__init__.py` - Package initialization (currently contains placeholder code)
  - `py.typed` - PEP 561 marker file indicating the package supports type hints
- `pyproject.toml` - Project configuration and dependencies (uses uv as build backend)
- `uv.lock` - Locked dependency versions for reproducible builds

## Dependencies

- **lanelet2** (>=1.2.2) - Core library for working with Lanelet2 map format
- Python 3.10 or higher is required

## Architecture Notes

This is a converter package designed to transform Lanelet2 map data (commonly used in Autoware and other autonomous driving stacks) into the OpenDRIVE format (an open standard for road network descriptions). The actual conversion logic needs to be implemented in the main package directory.

## Development Guidelines

1. Type hints should be used throughout the codebase (package includes py.typed marker)
2. Follow Python 3.10 syntax and features
3. The package name uses hyphens externally (`autoware-lanelet2-to-opendrive`) but underscores internally (`autoware_lanelet2_to_opendrive`)
4. Use uv for all dependency management operations rather than pip directly

## Language Policy

**IMPORTANT**: All project content must be written in English.

This applies to:
- **Documentation**: All `.md` files, docstrings, README files, and user-facing documentation
- **Source Code**: All variable names, function names, class names, and code identifiers
- **Comments**: All inline comments, block comments, and code explanations
- **Commit Messages**: All git commit messages and PR descriptions
- **Issue and PR Discussions**: All GitHub issue reports and pull request communications

### Rationale:
- Ensures accessibility for the global open-source community
- Facilitates collaboration among international contributors
- Maintains consistency across the codebase
- Enables better integration with international tools and AI assistants
- Follows industry best practices for open-source projects

### Exceptions:
- Test data or fixtures that specifically require non-English text
- Localization files (if the project supports multiple languages in the UI)
- Citations or references to non-English sources (should include English translation)

## Git Operation Restrictions

**IMPORTANT**: The following dangerous git operations are STRICTLY PROHIBITED for safety:

### Forbidden Commands:
- `git push --force` or `git push -f` (destroys history)
- `git push --force-with-lease` (can still overwrite others' work)
- `git push origin master` or `git push origin main` (direct push to protected branches)
- `git commit --no-verify` (bypasses pre-commit hooks and safety checks)
- `git push --no-verify` (bypasses push hooks and safety checks)

### Safe Alternatives:
- Use `git push` (normal push) - will fail safely if there are conflicts
- Use `git pull --rebase` followed by `git push` to handle conflicts
- Create pull requests instead of direct pushes to main/master
- Always let pre-commit hooks run to maintain code quality

### Exception Handling:
If a push is rejected due to conflicts or hook failures:
1. **DO NOT** use --force or --no-verify flags
2. Fix the underlying issue (resolve conflicts, fix formatting, etc.)
3. Use `git pull --rebase` to update your branch
4. Re-run tests and hooks to ensure everything passes
5. Use normal `git push` after issues are resolved

### Rationale:
- Force pushes can destroy other developers' work
- Bypassing hooks can introduce bugs, formatting issues, or security problems
- Direct pushes to main branches bypass code review processes
- These restrictions ensure code quality and collaborative safety

## Claude Code Settings Configuration

**IMPORTANT**: This CLAUDE.md file should be reflected in the Claude Code settings configuration file to enforce these guidelines automatically.

### Settings File Location:
`.claude/settings.local.json` (if it exists, otherwise create `.claude/settings.json`)

### Required Configuration:
The settings file should include these git operation restrictions using the permissions system to prevent dangerous commands:

```json
{
  "permissions": {
    "allow": [
      "Bash(git add:*)",
      "Bash(git commit:*)",
      "Bash(git push)",
      "Bash(git pull:*)",
      "... other allowed commands ..."
    ],
    "deny": [
      "Bash(git push --force:*)",
      "Bash(git push -f:*)",
      "Bash(git push --force-with-lease:*)",
      "Bash(git push origin master:*)",
      "Bash(git push origin main:*)",
      "Bash(git commit --no-verify:*)",
      "Bash(git push --no-verify:*)"
    ],
    "ask": []
  }
}
```

**Note**: The Claude Code settings schema only supports the `permissions` system. Custom fields like `git.prohibitedCommands` are not supported by the official schema.

### Instructions for Claude:
1. Always check `.claude/settings.local.json` for project-specific configurations
2. The `permissions.deny` list prevents execution of dangerous git commands
3. Only use `git push` (without flags) for safe pushes that respect remote conflicts
4. Suggest safe alternatives when dangerous commands are requested
5. Ensure pre-commit hooks always run (never bypass with --no-verify)
6. Follow the development guidelines specified in this CLAUDE.md file

### Enforcement:
- `.claude/settings.local.json` now contains explicit deny rules for dangerous git operations
- These settings will prevent Claude from executing prohibited commands
- The permissions system provides automatic enforcement without requiring manual checks
