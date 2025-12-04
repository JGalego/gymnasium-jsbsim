# Gymnasium-JSBSim Documentation

This directory contains the Sphinx documentation for Gymnasium-JSBSim.

## Building the Documentation

### Prerequisites

Install the documentation dependencies:

```bash
pip install -r ../requirements-dev.txt
```

### Build HTML Documentation

```bash
make html
```

The generated HTML files will be in `build/html/`. Open `build/html/index.html` in your browser to view the documentation.

### Other Build Formats

```bash
make latexpdf  # Build PDF documentation (requires LaTeX)
make epub      # Build EPUB documentation
make help      # Show all available targets
```

## Cleaning Build Files

```bash
make clean
```

## Documentation Structure

- `source/conf.py` - Sphinx configuration
- `source/index.rst` - Documentation homepage
- `source/modules.rst` - API reference
- `source/_static/` - Static files (CSS, images, etc.)
- `build/` - Generated documentation (gitignored)

## Hosting on GitHub Pages

To publish the documentation on GitHub Pages:

1. Build the documentation: `make html`
2. Create a `gh-pages` branch
3. Copy `build/html/*` to the root of `gh-pages` branch
4. Push to GitHub
5. Enable GitHub Pages in repository settings

Alternatively, use GitHub Actions to automate this process.
