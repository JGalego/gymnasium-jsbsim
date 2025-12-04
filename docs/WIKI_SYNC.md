# GitHub Wiki Sync Guide

This document explains how to sync your Markdown documentation to your GitHub Wiki.

## Prerequisites

1. Enable the Wiki for your repository (in GitHub Settings > Features)
2. Initialize the wiki by creating at least one page through the GitHub web interface

## Method 1: Automated GitHub Actions (Recommended)

The workflow in `.github/workflows/sync-wiki.yml` automatically syncs docs to the wiki on every push to master.

**Setup:**
1. Enable GitHub Actions in your repository
2. The workflow will run automatically when you push changes to:
   - Python files in `gymnasium_jsbsim/`
   - Markdown files in `docs/markdown/`
   - Source files in `docs/source/`

**What it does:**
- Generates fresh Markdown docs
- Copies `index.md` → Wiki `Home` page
- Copies `modules.md` → Wiki `API-Reference` page
- Commits and pushes changes to the wiki

## Method 2: Manual Script

Run the script manually when you want to update the wiki:

```bash
./scripts/sync-wiki.sh
```

**What it does:**
1. Clones or updates the wiki repository
2. Generates fresh Markdown documentation
3. Copies files to the wiki
4. Commits and pushes changes

## Method 3: Manual Copy-Paste

For one-time updates or custom organization:

1. **Clone the wiki repository:**
   ```bash
   git clone https://github.com/JGalego/gymnasium-jsbsim.wiki.git
   ```

2. **Generate docs:**
   ```bash
   cd docs
   make markdown
   ```

3. **Copy files:**
   ```bash
   # The wiki Home page
   cp docs/markdown/index.md ../gymnasium-jsbsim.wiki/Home.md
   
   # API Reference
   cp docs/markdown/modules.md ../gymnasium-jsbsim.wiki/API-Reference.md
   ```

4. **Commit and push:**
   ```bash
   cd ../gymnasium-jsbsim.wiki
   git add .
   git commit -m "Update documentation"
   git push
   ```

## Wiki Structure

The wiki will have the following pages:

- **Home** (`index.md`) - Main documentation page with quick start
- **API Reference** (`modules.md`) - Complete API documentation

## Customization

You can modify the sync behavior by editing:
- `.github/workflows/sync-wiki.yml` - GitHub Actions workflow
- `scripts/sync-wiki.sh` - Manual sync script
- `docs/source/index.rst` - Main page content
- `docs/source/modules.rst` - API reference structure

## Notes

- Wiki changes are **overwritten** on sync - don't edit directly in the wiki
- Make all documentation changes in the main repository
- The wiki repository is separate from your main repository
- Wiki pages use standard GitHub Markdown rendering
