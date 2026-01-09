# Development Guide

This guide is for developers who want to contribute to the `autoware-lanelet2-to-opendrive` project.

## Development Environment Setup

### Prerequisites

- Python 3.10 or higher
- uv (version 0.9.7+)
- Git

### Setting Up Your Environment

1. **Fork and clone the repository**:
```bash
git clone https://github.com/YOUR_USERNAME/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

2. **Create a virtual environment and install dependencies**:
```bash
uv venv
source .venv/bin/activate  # On Linux/macOS
# or
.venv\Scripts\activate  # On Windows

uv sync
```

3. **Install the package in editable mode**:
```bash
uv pip install -e .
```

4. **Install pre-commit hooks** (recommended):
```bash
pre-commit install
```

## Project Structure

```
autoware_lanelet2_to_opendrive/
├── src/
│   └── autoware_lanelet2_to_opendrive/  # Main package
│       ├── __init__.py                   # Package initialization
│       └── py.typed                      # Type hints marker
├── docs/                                 # Documentation
├── examples/                             # Example scripts
├── test/                                 # Tests
├── pyproject.toml                        # Project configuration
├── mkdocs.yml                           # Documentation config
└── uv.lock                              # Locked dependencies
```

## Development Workflow

### Using uv for Development

```bash
# Add a new dependency
uv add <package_name>

# Add a development dependency
uv add --dev <package_name>

# Run Python scripts
uv run python <script.py>

# Update dependencies
uv sync --refresh
```

### Code Style and Quality

This project follows Python best practices and enforces them through pre-commit hooks:

#### Coding Standards

- **Python Version**: Python 3.10+ syntax and features
- **Type Hints**: All functions and methods must include type annotations (package includes `py.typed` marker)
- **Docstrings**: Use Google-style docstrings for all public modules, classes, and functions
- **Naming Conventions**:
  - Use `snake_case` for functions and variables
  - Use `PascalCase` for class names
  - Package name uses hyphens externally (`autoware-lanelet2-to-opendrive`) but underscores internally (`autoware_lanelet2_to_opendrive`)
- **Code Formatting**: Automatically enforced by Ruff formatter
- **Linting**: Ruff linter with auto-fix enabled
- **Type Checking**: mypy with `--ignore-missing-imports`
- **Import Organization**: Imports should be organized and sorted

#### Pre-commit Hooks

The project uses pre-commit hooks to ensure code quality. These are automatically run on commit and include:

- Trailing whitespace removal
- End-of-file fixer
- YAML/TOML validation
- Large file checks
- Merge conflict detection
- Debug statement detection
- Ruff formatting and linting
- mypy type checking
- pytest test execution

**Important**: Never bypass pre-commit hooks with `--no-verify` unless absolutely necessary and approved by maintainers.

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=autoware_lanelet2_to_opendrive
```

## Contributing

### Contribution Guidelines

1. **Create an issue** first to discuss your proposed changes
2. **Fork the repository** and create a feature branch
3. **Write tests** for your changes
4. **Ensure all tests pass** and code is properly formatted
5. **Submit a pull request** with a clear description

### Commit Message Convention

Follow conventional commit format:

- `feat:` - New features
- `fix:` - Bug fixes
- `docs:` - Documentation changes
- `test:` - Test additions or changes
- `refactor:` - Code refactoring
- `chore:` - Maintenance tasks

### Git Safety

!!! warning "Important Git Restrictions"
    For safety and code quality, the following git operations are **strictly prohibited**:

    - `git push --force` or `git push -f`
    - `git push --force-with-lease`
    - `git commit --no-verify` or `git push --no-verify`
    - Direct pushes to main/master branches

    Always use normal `git push` and let pre-commit hooks run to maintain code quality.

## Documentation

### Building Documentation Locally

```bash
# Install MkDocs if not already installed
uv add --dev mkdocs mkdocs-material mkdocstrings[python]

# Serve documentation locally
mkdocs serve
```

Visit `http://127.0.0.1:8000` to view the documentation.

### Documentation Structure

- `docs/index.md` - Home page
- `docs/installation.md` - Installation guide
- `docs/usage.md` - Usage examples
- `docs/api.md` - API reference (auto-generated)
- `docs/development.md` - This file
- `docs/signals.md` - Signal handling documentation

### Writing Documentation

- Use clear, concise language
- Include code examples where appropriate
- Add cross-references to related sections
- Use admonitions (notes, warnings, etc.) to highlight important information

## Architecture

### Design Principles

- **Type Safety**: Full type hints for better IDE support and error catching
- **Modularity**: Clear separation of concerns
- **Testability**: Design for easy testing
- **Documentation**: Well-documented code and APIs

### Conversion Pipeline

The conversion process typically involves:

1. **Parsing** - Read and validate Lanelet2 input
2. **Transformation** - Convert data structures
3. **Generation** - Create OpenDRIVE output
4. **Validation** - Ensure output correctness

## Testing Strategy

- **Unit tests** - Test individual components
- **Integration tests** - Test conversion workflows
- **Validation tests** - Verify output format compliance

## Release Process

This section describes the process for creating and publishing new releases.

### Version Management

The project follows [Semantic Versioning](https://semver.org/) (SemVer):

- **MAJOR** version for incompatible API changes
- **MINOR** version for backwards-compatible functionality additions
- **PATCH** version for backwards-compatible bug fixes

Version is defined in `pyproject.toml` under `[project]` section.

### Release Checklist

Before creating a release, ensure:

1. ✅ All tests pass locally and in CI
2. ✅ Documentation is up to date
3. ✅ CHANGELOG.md is updated (if exists) with release notes
4. ✅ Version number is bumped in `pyproject.toml`
5. ✅ All changes are merged to the main branch

### Creating a Release

1. **Update version number** in `pyproject.toml`:
   ```toml
   [project]
   version = "0.2.0"  # Update this
   ```

2. **Create a release commit**:
   ```bash
   git add pyproject.toml
   git commit -m "chore: bump version to 0.2.0"
   git push origin main
   ```

3. **Create a git tag**:
   ```bash
   git tag -a v0.2.0 -m "Release version 0.2.0"
   git push origin v0.2.0
   ```

4. **Create a GitHub Release**:
   - Go to the repository's Releases page
   - Click "Draft a new release"
   - Select the tag you just created
   - Add release notes describing changes, bug fixes, and new features
   - Publish the release

### Publishing to PyPI

If the package is published to PyPI:

1. **Build the package**:
   ```bash
   uv build
   ```

2. **Upload to PyPI** (requires maintainer access):
   ```bash
   uv publish
   ```

   Or use twine:
   ```bash
   twine upload dist/*
   ```

### Post-Release

After releasing:

1. Verify the release appears on GitHub
2. Test installation from PyPI (if published)
3. Update any dependent projects
4. Announce the release in relevant channels

### Hotfix Releases

For urgent bug fixes:

1. Create a hotfix branch from the release tag
2. Apply the minimal fix
3. Update version to next patch version
4. Follow the release process
5. Merge hotfix back to main

## Getting Help

- Check existing issues on GitHub
- Join discussions in pull requests
- Reach out to maintainers

## Resources

### Related Projects

- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Source format library
- [Autoware](https://github.com/autowarefoundation/autoware) - Target platform

### Standards and Specifications

- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/)
- [PEP 561](https://www.python.org/dev/peps/pep-0561/) - Type hints in packages
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)

## License

Check the repository for license information.
