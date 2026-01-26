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

3. **Run manual checks before committing** if there's any doubt:
   ```bash
   pre-commit run --all-files
   ```

4. **If a commit fails due to lint errors**:
   - Review the error output
   - Fix the issues or let pre-commit auto-fix them
   - Stage the fixes with `git add`
   - Retry the commit (without `--no-verify`)

5. **Common pre-commit hooks in this project may include**:
   - **black**: Python code formatting
   - **isort**: Import statement sorting
   - **flake8**: Python linting
   - **mypy**: Static type checking
   - **trailing-whitespace**: Remove trailing whitespace
   - **end-of-file-fixer**: Ensure files end with newline
   - **check-yaml**: Validate YAML syntax

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

### Rationale

- **Consistency**: Standardized format across all PRs and issues
- **Quality**: Ensures all necessary information is provided
- **Efficiency**: Reviewers can quickly understand changes
- **Documentation**: PRs serve as historical record of design decisions
- **Professional standards**: Follows industry best practices for open-source projects

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
