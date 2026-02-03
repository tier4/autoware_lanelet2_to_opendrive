# Release Notes

This page contains the release history and changelog for the Autoware Lanelet2 to OpenDRIVE converter.

## Version History

### v1.2.2 (2025-02-03)

**🐛 Bug Fixes**
- Fixed dataclass field ordering issue by making tolerance field keyword-only

**📝 Documentation**
- Added known limitations documentation
- Updated API reference documentation

### v1.2.1 (2025-02-02)

**✨ Features**
- Improved lane connectivity handling for branching scenarios
- Enhanced lane link elements generation

**🐛 Bug Fixes**
- Fixed connecting road lane successor issue (#128)
- Fixed lane connectivity branching issue (#130)
- Fixed lane link elements issue (#124)

### v1.2.0 (2025-01-30)

**✨ Features**
- Added signal reference elements support
- Added traffic rule attribute support
- Improved CARLA compatibility with exclude_non_junction_signals option

**♻️ Refactoring**
- Moved configuration to package for better organization
- Improved preprocessing operations structure

**📝 Documentation**
- Added comprehensive signal documentation
- Updated usage guide with Hydra configuration examples

### v1.1.0 (2025-01-15)

**✨ Features**
- Added preprocessing commands for lanelet manipulation
  - Merge operations
  - Remove operations
  - Replace operations
  - Validate operations
  - Move point operations
  - Delete point operations
- Added stop line support
- Added speed limit handling
- Implemented road rule tag support

**🛠️ Infrastructure**
- Added PR auto-check workflow
- Added GitHub issue and PR templates
- Configured automatic version bumping

### v1.0.0 (2025-01-01)

**🎉 Initial Release**

- Core conversion from Lanelet2 (OSM format) to OpenDRIVE
- MGRS coordinate projection support
- B-spline geometry fitting for smooth roads
- Lane connectivity and successor/predecessor relationships
- Junction handling
- Basic signal conversion

---

## Release Note Format

Each release note follows this structure:

### Version Number and Date

`### vX.Y.Z (YYYY-MM-DD)`

### Change Categories

- **🎉 Initial Release**: First public release
- **✨ Features**: New features and enhancements
- **🐛 Bug Fixes**: Bug fixes and corrections
- **♻️ Refactoring**: Code improvements without changing functionality
- **🛠️ Infrastructure**: CI/CD, tooling, and development workflow improvements
- **📝 Documentation**: Documentation updates
- **⚡️ Performance**: Performance improvements
- **🔒 Security**: Security fixes
- **⚠️ Deprecations**: Deprecated features
- **💥 Breaking Changes**: Changes that break backward compatibility

### Change Description

Each change should include:
- A clear, concise description of what changed
- Issue/PR reference numbers when applicable
- Impact on users (for breaking changes)

---

## How to Generate Release Notes

### Automatic Generation

When creating a new release on GitHub:

1. Go to [Releases](https://github.com/tier4/autoware_lanelet2_to_opendrive/releases)
2. Click "Draft a new release"
3. Choose a tag (e.g., `v1.3.0`)
4. Click "Generate release notes" to auto-generate from PRs
5. Organize the changes into categories using the emoji format above
6. Publish the release

The CI/CD pipeline will automatically:
- Build the documentation with MkDocs
- Generate a PDF version of the release notes
- Attach the PDF to the GitHub release
- Deploy the HTML documentation to GitHub Pages

### Manual Updates

To update this file manually:

1. Add a new version section at the top
2. Organize changes by category with emoji indicators
3. Include relevant PR/issue references
4. Commit and push the changes

---

## Download Options

- **HTML**: View online at [GitHub Pages](https://tier4.github.io/autoware_lanelet2_to_opendrive/release-notes/)
- **PDF**: Download from [GitHub Releases](https://github.com/tier4/autoware_lanelet2_to_opendrive/releases)
- **Source**: View the [source markdown](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/docs/release-notes.md)

---

## Contributing

If you notice any missing or incorrect information in the release notes, please:

1. Open an issue on [GitHub](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)
2. Submit a PR with corrections
3. Follow the [Contributing Guidelines](development.md)
