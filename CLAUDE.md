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

## Pre-commit Hooks and Lint Checking

**CRITICAL**: This project uses pre-commit hooks to ensure code quality and prevent lint errors. All commits must pass these checks before being pushed.

### Installation

Pre-commit hooks must be installed in your local repository before making any commits:

```bash
# Install pre-commit (if not already installed)
pip install pre-commit

# Install the git hook scripts
pre-commit install
```

### Usage

#### Automatic Checking (Recommended)
Once installed, pre-commit hooks run automatically on every `git commit`. The commit will be blocked if any checks fail.

#### Manual Checking
You can manually run pre-commit checks before committing:

```bash
# Run on all files
pre-commit run --all-files

# Run on staged files only
pre-commit run

# Run on specific files
pre-commit run --files <file1> <file2>
```

### Common Lint Errors and Fixes

If pre-commit hooks fail:

1. **Review the error messages** - They usually indicate what needs to be fixed
2. **Let pre-commit auto-fix when possible** - Many formatters (like black, isort) automatically fix issues
3. **Stage the auto-fixed changes**:
   ```bash
   git add -u
   ```
4. **Retry the commit**:
   ```bash
   git commit
   ```

### Important Notes for Claude Code

**MANDATORY**: When working with this repository through Claude Code:

1. **Always install pre-commit hooks** at the start of any work session:
   ```bash
   pre-commit install
   ```

2. **Never bypass pre-commit hooks** with `--no-verify` flag (this is already prohibited in Git Operation Restrictions)

3. **CRITICAL: Auto-format code BEFORE committing** to prevent CI/CD failures:
   ```bash
   # Step 1: Run pre-commit on all files to auto-fix formatting issues
   pre-commit run --all-files

   # Step 2: If files were modified, stage the changes
   git add -u

   # Step 3: Now commit (pre-commit will pass because code is already formatted)
   git commit -m "your message"
   ```

   **Rationale**: GitHub Actions fails when pre-commit hooks modify files (exit code 1). By running `pre-commit run --all-files` before committing, formatters like `ruff-format` will fix issues locally first, preventing CI failures.

4. **Automated workflow for code generation and PR creation**:

   When generating code and creating a PR, **ALWAYS** follow this sequence:

   ```bash
   # 1. Make code changes (via Write/Edit tools)

   # 2. Auto-format all files
   pre-commit run --all-files

   # 3. Stage all changes (including formatter modifications)
   git add -u

   # 4. Commit with proper message
   git commit -m "feat: your feature description

   Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>"

   # 5. Push to remote
   git push -u origin branch-name

   # 6. Create PR using gh CLI
   gh pr create --title "..." --body "..."
   ```

5. **If a commit fails due to lint errors**:
   - Review the error output
   - Run `pre-commit run --all-files` to let formatters auto-fix
   - Stage the fixes with `git add -u`
   - Retry the commit (without `--no-verify`)

   **NEVER** bypass hooks with `--no-verify` even if the commit fails multiple times.

6. **Common pre-commit hooks in this project**:
   - **ruff**: Python linting (checks code style, imports, etc.)
   - **ruff-format**: Python code formatting (auto-fixes formatting issues)
   - **mypy**: Static type checking
   - **trailing-whitespace**: Remove trailing whitespace
   - **end-of-file-fixer**: Ensure files end with newline
   - **check-yaml**: Validate YAML syntax
   - **check-toml**: Validate TOML syntax
   - **debug-statements**: Detect debug statements like `breakpoint()`
   - **mixed-line-ending**: Detect mixed line endings

7. **Understanding pre-commit hook results**:
   - **Passed**: Hook found no issues
   - **Failed (files were modified by this hook)**: Hook auto-fixed issues - you MUST stage changes with `git add -u` and retry
   - **Failed (errors found)**: Manual fixes required - review output and fix issues

### Rationale

- **Prevents CI/CD failures**: Catching lint errors locally before pushing
- **Maintains code quality**: Consistent formatting and style across the codebase
- **Saves time**: Faster feedback than waiting for GitHub Actions
- **Collaborative safety**: Ensures all contributors follow the same standards
- **Professional development**: Industry best practice for Python projects

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

## GitHub Issue and Pull Request Templates

**IMPORTANT**: This project uses GitHub issue and pull request templates to maintain consistency and quality in project communications.

### Template Files

- **Pull Request Template**: `.github/PULL_REQUEST_TEMPLATE.md`
- **Bug Report Template**: `.github/ISSUE_TEMPLATE/bug_report.md`
- **Feature Request Template**: `.github/ISSUE_TEMPLATE/feature_request.md`
- **Template Configuration**: `.github/ISSUE_TEMPLATE/config.yml`

### Style Reference

See [PR #133](https://github.com/tier4/autoware_lanelet2_to_opendrive/pull/133) as an example of the project's established PR description style with emoji indicators (🐛 ✨ ♻️ 🔍 🛠️ ✅ 📊 ⚠️ 📝) and comprehensive documentation.

### Instructions for Claude Code

**MANDATORY**: When creating PRs or issues through Claude Code:

1. **Read the appropriate template file first**
   - For PRs: Read `.github/PULL_REQUEST_TEMPLATE.md` and use its structure
   - For bug reports: Read `.github/ISSUE_TEMPLATE/bug_report.md` and follow its format
   - For feature requests: Read `.github/ISSUE_TEMPLATE/feature_request.md` and follow its format

2. **Follow template structure exactly**
   - Fill out all required sections in the template
   - Delete optional sections marked as "delete if not applicable"
   - Use emoji indicators as specified in the template
   - Include code examples with syntax highlighting where appropriate
   - Use checkboxes for lists where indicated

3. **Maintain Language Policy**
   - All PR/issue content must be in English
   - Use clear, professional technical writing

4. **Ensure pre-commit compliance**
   - All pre-commit checks must pass before creating the PR
   - Include `Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>` if assisted by Claude

5. **Add version bump label**
   - **MANDATORY**: Every PR must have exactly one version bump label
   - Choose the appropriate label based on semantic versioning:
     - `bump patch`: Bug fixes, documentation updates, minor changes (0.0.X)
     - `bump minor`: New features, non-breaking enhancements (0.X.0)
     - `bump major`: Breaking changes, major refactoring (X.0.0)
   - Add the label when creating the PR using: `gh pr create --label "bump patch"`
   - **Important**: The `Check Version Bump Label` GitHub Action will fail if no version bump label is present
   - **Default recommendation**: Use `bump patch` if uncertain about the appropriate level

### Rationale

- **Consistency**: Standardized format across all PRs and issues
- **Quality**: Ensures all necessary information is provided
- **Efficiency**: Reviewers can quickly understand changes
- **Documentation**: PRs serve as historical record of design decisions
- **Professional standards**: Follows industry best practices for open-source projects

## GitHub Actions and Automated PR Creation

**CRITICAL**: This section describes the mandatory workflow for creating PRs to ensure all GitHub Actions checks pass.

### Problem: Pre-commit Formatting in CI/CD

GitHub Actions runs `pre-commit run --all-files` in the `lint-and-format` job. If any hook modifies files (e.g., `ruff-format` reformats code), the job **fails with exit code 1** even though the formatting is correct. This is because:

1. Pre-commit hooks modify files during CI
2. Modified files indicate the committed code was not properly formatted
3. GitHub Actions interprets this as a failure

**Example failure from PR #160**:
```
ruff-format..............................................................Failed
- hook id: ruff-format
- files were modified by this hook

1 file reformatted, 50 files left unchanged
##[error]Process completed with exit code 1.
```

### Solution: Pre-format Code Before Committing

**MANDATORY**: Claude Code must **ALWAYS** run formatters locally before committing code, ensuring that the code pushed to GitHub is already formatted correctly.

### Automated PR Creation Workflow

When creating a PR (manually or through automation), **STRICTLY FOLLOW** this sequence:

```bash
# 1. Complete all code changes using Write/Edit tools

# 2. Format all files before committing
#    This step is CRITICAL to prevent CI failures
pre-commit run --all-files

# 3. Check if formatters modified any files
#    If "files were modified by this hook" appears, proceed to step 4
#    If all hooks passed, proceed to step 5

# 4. Stage formatter changes
git add -u

# 5. Commit with proper message (hooks will pass now)
git commit -m "feat: your feature description

Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>"

# 6. Push to remote branch
git push -u origin feature-branch-name

# 7. Create PR using gh CLI with template and version bump label
gh pr create --title "..." --body "..." --label "bump patch"
# Note: Choose the appropriate label:
#   - "bump patch" for bug fixes and minor changes
#   - "bump minor" for new features
#   - "bump major" for breaking changes
```

### Detailed Step-by-Step Instructions for Claude Code

When you are asked to create a PR or when you autonomously decide to create a PR:

1. **After writing/editing code files**:
   - Use Write/Edit tools to make all necessary code changes
   - Do NOT commit yet

2. **Run pre-commit formatters**:
   ```bash
   pre-commit run --all-files
   ```
   - This will auto-format code using `ruff-format`, `ruff`, etc.
   - Watch for "files were modified by this hook" messages
   - If any hook reports modifications, proceed to step 3
   - If all hooks pass without modifications, proceed to step 4

3. **Stage formatter modifications** (if any hooks modified files):
   ```bash
   git add -u
   ```
   - This stages all tracked file modifications
   - Includes formatting changes made by pre-commit hooks

4. **Commit with formatted code**:
   ```bash
   git commit -m "feat: implement feature X

   Detailed explanation of changes.

   Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>"
   ```
   - Pre-commit hooks run again but will pass immediately
   - Code is already formatted from step 2

5. **Push to remote**:
   ```bash
   git push -u origin feature-branch-name
   ```

6. **Create PR with version bump label**:
   - Read `.github/PULL_REQUEST_TEMPLATE.md` first
   - Use `gh pr create` with appropriate title, body, and **version bump label**
   - Follow template structure and emoji conventions
   - **MANDATORY**: Add one of the following labels:
     - `--label "bump patch"` for bug fixes and minor changes (default if uncertain)
     - `--label "bump minor"` for new features
     - `--label "bump major"` for breaking changes
   - Example: `gh pr create --title "feat: add new feature" --body "..." --label "bump minor"`

### Common Errors and Solutions

#### Error: "ruff-format failed - files were modified by this hook"

**Cause**: Code was not formatted before committing

**Solution**:
```bash
# Run pre-commit to format
pre-commit run --all-files

# Stage the formatting changes
git add -u

# Create a new commit OR amend if not pushed yet
git commit -m "style: apply ruff formatting"

# Push
git push
```

#### Error: "lint-and-format job failed in GitHub Actions"

**Cause**: Committed code does not pass pre-commit checks

**Solution**:
1. Pull the branch locally
2. Run `pre-commit run --all-files`
3. Fix any remaining errors manually
4. Stage changes with `git add -u`
5. Commit and push
6. CI will re-run automatically

### Rationale

- **Prevents CI failures**: Ensures code is formatted before reaching GitHub Actions
- **Saves time**: Avoids failed CI runs and resubmissions
- **Consistency**: All code follows project formatting standards
- **Professionalism**: Clean commit history without formatting-only commits
- **Automation-friendly**: Works seamlessly with Claude Code's automated workflows

### Integration with Commit Workflow

This formatting requirement is integrated into the commit workflow described in the Bash tool's "Committing changes with git" section. The sequence is:

1. Write/Edit code → 2. **Format with pre-commit** → 3. Stage changes → 4. Commit → 5. Push

**Step 2 is mandatory and must never be skipped.**

### Automated Formatting in GitHub Actions

**IMPORTANT**: The `.github/workflows/claude.yml` workflow includes automatic formatting as a safety net.

#### How It Works

When Claude Code GitHub Actions runs:

1. **Setup Phase**:
   - Checkout repository with full git history
   - Install Python, uv, system dependencies
   - Install project dependencies with `uv sync --dev`
   - Install pre-commit hooks with `uv run pre-commit install`

2. **Claude Code Execution**:
   - Claude Code runs with write permissions (`contents: write`, `pull-requests: write`)
   - Claude Code should follow CLAUDE.md guidelines and format code before committing
   - Creates commits and pushes to branch

3. **Auto-format Safety Net** (runs after Claude Code):
   - Runs `uv run pre-commit run --all-files` on all code
   - Detects if any files were modified by formatters
   - If changes detected:
     - Stages changes with `git add -u`
     - Commits with message: `"style: auto-format code with ruff-format"`
     - Pushes to the same branch
   - If no changes: Reports success, no action needed

4. **Summary**:
   - GitHub Actions summary shows whether formatting was needed
   - Provides transparency on what was done

#### Permissions

The workflow has these permissions:
- `contents: write` - Allows committing formatting changes
- `pull-requests: write` - Allows creating/updating PRs
- `actions: read` - Allows reading CI results

#### Benefits

- ✅ **Safety net**: Even if Claude Code misses formatting, it's automatically fixed
- ✅ **CI/CD compliance**: Ensures all code passes `lint-and-format` checks
- ✅ **Zero manual intervention**: Fully automated formatting pipeline
- ✅ **Transparent**: GitHub Actions summary shows what was done
- ✅ **Best practices**: Follows the same workflow as local development

#### Configuration File

The automated formatting is configured in:
- **Workflow**: `.github/workflows/claude.yml`
- **Pre-commit config**: `.pre-commit-config.yaml`
- **Project config**: `pyproject.toml` (for ruff settings)

#### What This Means for Claude Code

When Claude Code runs in GitHub Actions:
1. **Primary responsibility**: Claude Code should still format code before committing (following CLAUDE.md guidelines)
2. **Backup protection**: If formatting is missed, the workflow automatically fixes it
3. **No manual fixes needed**: Users don't need to manually fix formatting issues in PRs created by Claude Code
4. **Clean history**: Formatting commits are clearly marked and attributed

## Link Checker

**IMPORTANT**: This project uses automated link checking to ensure all URLs in documentation and code remain valid.

### Workflow Configuration

The link checker workflow is configured in `.github/workflows/link-checker.yml` and runs:

1. **On pull requests**: Checks links in all modified files
2. **Weekly schedule**: Runs every Sunday at 00:00 UTC to catch external link rot
3. **Manual trigger**: Can be triggered manually via workflow_dispatch

### How It Works

The workflow uses [lychee](https://github.com/lycheeverse/lychee), a fast Rust-based link checker:

- **Scans multiple file types**: Markdown (.md), Python (.py), YAML (.yml, .yaml), TOML (.toml), HTML (.html)
- **Caches results**: Uses GitHub Actions cache to avoid rate-limiting (1-day TTL)
- **Creates issues**: Automatically creates GitHub issues when broken links are found (scheduled runs only)
- **Generates reports**: Uploads detailed reports as workflow artifacts (30-day retention)

### Configuration Files

#### `.lycheeignore`
Contains regex patterns for URLs to exclude from checking:
- Localhost and local development URLs
- Example/placeholder domains (example.com, example.org, etc.)
- Authentication-required pages (GitHub settings, notifications, etc.)
- Rate-limited APIs
- Social media platforms with unstable link checking

#### `lychee.toml`
Main configuration file with settings:
- **Cache**: Enabled with 1-day max age
- **Retries**: Max 3 retries with 20s timeout
- **Redirects**: Follows up to 10 redirects
- **User Agent**: Identifies as Lychee link checker
- **Schemes**: Only checks http and https URLs
- **Excluded paths**: Ignores .git, node_modules, __pycache__, etc.

### Local Usage

You can run link checking locally before committing:

```bash
# Install lychee (if not installed)
# On macOS
brew install lychee

# On Linux
cargo install lychee-cli

# Run link checker on all files
lychee --config lychee.toml .

# Run on specific files or directories
lychee docs/
lychee README.md

# Use verbose mode for debugging
lychee --verbose .
```

### Adding Exclusions

If you encounter false positives or need to exclude specific URLs:

1. **Add to `.lycheeignore`**: For regex patterns (e.g., all LinkedIn URLs)
   ```
   https?://(www\.)?linkedin\.com/.*
   ```

2. **Add to `lychee.toml`**: For exact URLs or path exclusions
   ```toml
   exclude = [
       "https://example.com/api/endpoint",
   ]
   ```

### Troubleshooting

#### Link checker reports false positives

**Solution**: Add the URL pattern to `.lycheeignore` or `lychee.toml` exclusions

#### Rate limiting errors

**Solution**: The workflow uses caching to mitigate rate limiting. For local runs, wait and retry later.

#### Authentication-required URLs

**Solution**: Links that require authentication should be excluded in `.lycheeignore`

### Rationale

- **Prevents broken links**: Catches link rot before it affects users
- **Improves documentation quality**: Ensures all references remain accessible
- **Saves time**: Automated checking is faster than manual verification
- **Professional standards**: Maintains high-quality documentation and references
- **Early detection**: Weekly scans catch external link changes proactively

## Git Operation Restrictions

**IMPORTANT**: The following dangerous git operations are STRICTLY PROHIBITED for safety:

### Forbidden Commands:
- `git push --force` or `git push -f` (destroys history)
- `git push --force-with-lease` (can still overwrite others' work)
- `git push origin master` or `git push origin main` (direct push to protected branches)
- `git commit --no-verify` (bypasses pre-commit hooks and safety checks)
- `git push --no-verify` (bypasses push hooks and safety checks)
- `git rebase` (rewrites history, requires force push, complicates collaboration)
- `git pull --rebase` (same issues as rebase)

### Safe Alternatives:
- Use `git push` (normal push) - will fail safely if there are conflicts
- Use `git merge origin/master` to integrate upstream changes (preserves history)
- Use `git pull` (without --rebase) to fetch and merge
- Create pull requests instead of direct pushes to main/master
- Always let pre-commit hooks run to maintain code quality

### Exception Handling:
If a push is rejected due to conflicts or hook failures:
1. **DO NOT** use --force, --no-verify, or rebase
2. Fix the underlying issue (resolve conflicts, fix formatting, etc.)
3. Use `git merge origin/master` to integrate upstream changes
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

## Constants Configuration

**IMPORTANT**: All magic numbers and configurable parameters must be centralized in the `config.py` module using dataclasses.

### Architecture

The project uses a dataclass-based configuration system to centralize all constants:

- **`config.py`**: Central configuration module containing all constants organized by functional area
- **Dataclass-based**: Constants are grouped into frozen dataclasses for type safety and immutability
- **Global instance**: A `DEFAULT_CONFIG` instance provides easy access throughout the codebase

### Constants Organization

Constants are organized into logical groups:

1. **GeometryConstants**: Geometry calculations and numerical stability
   - `epsilon`: Tolerance for numerical stability (1e-10)
   - `point_distance_threshold`: Minimum distance between distinct points (0.001)

2. **SplineConstants**: B-spline fitting and interpolation
   - `speed_epsilon`: Tolerance for speed/velocity checks (1e-12)
   - `hard_constraint_weight`: Weight for boundary constraints (80.0)
   - `soft_constraint_weight`: Weight for data fitting (20.0)
   - `knot_alpha_weight`: Weight for uniform knot distribution (2.0)
   - `knot_beta_weight`: Weight for curvature-adaptive knots (2.0)
   - `position_tolerance`: Endpoint position error tolerance (5.0)
   - `velocity_tolerance`: Endpoint velocity error tolerance (15.0)
   - `max_avg_error`: Maximum average fitting error (2.0)
   - `max_point_error`: Maximum single-point error (8.0)
   - `warn_percentile`: Error reporting percentile (95.0)

3. **PreprocessingConstants**: Lanelet preprocessing operations
   - `merge_tolerance_default`: Default merge tolerance (1e-3)
   - `replace_tolerance_default`: Default replacement tolerance (1e-3)
   - `validate_tolerance_default`: Default validation tolerance (1e-3)

### Usage Guidelines

#### When to Add a Constant

Add a constant to `config.py` when:
- A numeric value appears in multiple locations
- A threshold or tolerance value controls behavior
- A value might need tuning or adjustment
- A magic number lacks clear documentation

#### How to Use Constants

```python
from .config import DEFAULT_CONFIG

# Access geometry constants
if distance < DEFAULT_CONFIG.geometry.epsilon:
    # Handle near-zero case
    pass

# Access spline constants
spline = Splines(
    points,
    num_control_points=10,
)  # Spline class internally uses DEFAULT_CONFIG

# Access preprocessing constants
merged = merge_lanelets(
    lanelet_map,
    lanelet_ids,
    tolerance=DEFAULT_CONFIG.preprocessing.merge_tolerance_default
)
```

#### Adding New Constants

When adding new constants:

1. **Choose the appropriate dataclass** or create a new one if needed
2. **Add the constant with a descriptive name** using UPPER_SNAKE_CASE
3. **Provide a clear docstring** explaining what the constant controls
4. **Include the default value** and units if applicable
5. **Update this documentation** to reflect the new constant

Example:
```python
@dataclass(frozen=True)
class GeometryConstants:
    """Constants for geometry calculations."""

    epsilon: float = 1e-10  # Tolerance for numerical stability
    new_threshold: float = 0.5  # NEW: Description of what this controls
```

#### Modifying Dataclasses

When operation dataclasses (like `MergeOperation`, `ReplaceOperation`) need default values:

```python
@dataclass
class MergeOperation:
    """Configuration for merge operations."""

    lanelet_ids: List[int]
    validate: bool = True
    tolerance: float = None  # Will default to config value

    def __post_init__(self):
        """Set default tolerance from config if not specified."""
        if self.tolerance is None:
            self.tolerance = DEFAULT_CONFIG.preprocessing.merge_tolerance_default
```

### Benefits

- **Centralized configuration**: All constants in one location
- **Type safety**: Dataclasses provide type hints and validation
- **Immutability**: Frozen dataclasses prevent accidental modification
- **Discoverability**: IDE autocomplete shows all available constants
- **Documentation**: Each constant has clear documentation
- **Easy tuning**: Adjust parameters without searching the codebase

### Anti-Patterns to Avoid

❌ **DON'T** hardcode magic numbers in implementation files:
```python
if distance < 1e-10:  # Bad: magic number
    pass
```

✅ **DO** use constants from config:
```python
if distance < DEFAULT_CONFIG.geometry.epsilon:  # Good: documented constant
    pass
```

❌ **DON'T** define constants separately in multiple files:
```python
# geometry.py
EPSILON = 1e-10

# spline.py
EPSILON = 1e-10  # Bad: duplicated
```

✅ **DO** import from the central config:
```python
from .config import DEFAULT_CONFIG
# Use DEFAULT_CONFIG.geometry.epsilon everywhere
```

### Rationale

- **Prevents duplication**: Constants defined once, used everywhere
- **Improves maintainability**: Change a value in one place
- **Documents intent**: Clear names explain what values control
- **Type safety**: Dataclasses catch errors at development time
- **Professional code**: Industry best practice for configuration management
