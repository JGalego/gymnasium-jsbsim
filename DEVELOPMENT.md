# Development Setup

This guide helps you set up the development environment for gymnasium-jsbsim.

## Prerequisites

- Python 3.10 or higher
- JSBSim flight dynamics model installed ([installation guide](https://github.com/JSBSim-Team/jsbsim))

## Installation

1. Clone the repository:

```bash
git clone https://github.com/JGalego/gymnasium-jsbsim.git
cd gymnasium-jsbsim
```

2. Create a virtual environment:

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install the package in editable mode:

```bash
pip install -e .
```

4. Install development dependencies:

```bash
pip install -r requirements-dev.txt
```

5. Set up pre-commit hooks:

```bash
pre-commit install
```

## Running Tests

Run all tests:

```bash
pytest gymnasium_jsbsim/tests/ -v
```

Run with coverage:

```bash
pytest gymnasium_jsbsim/tests/ -v --cov=gymnasium_jsbsim --cov-report=html
```

Run specific test file:

```bash
pytest gymnasium_jsbsim/tests/test_environment.py -v
```

## Code Quality

### Formatting

Format code with Black:

```bash
black gymnasium_jsbsim/
```

Sort imports with isort:

```bash
isort --profile black gymnasium_jsbsim/
```

### Linting

Run Ruff linter:

```bash
ruff check gymnasium_jsbsim/
```

Fix auto-fixable issues:

```bash
ruff check --fix gymnasium_jsbsim/
```

### Type Checking

Run mypy:

```bash
mypy gymnasium_jsbsim/ --ignore-missing-imports --check-untyped-defs
```

### Run All Checks

Pre-commit will run all checks on staged files:

```bash
pre-commit run --all-files
```

## Continuous Integration

The CI pipeline runs automatically on push and pull requests. It includes:
- Testing on Python 3.10, 3.11, and 3.12
- Code linting with Ruff
- Code formatting checks with Black and isort
- Type checking with mypy
- Coverage reporting to Codecov

## Version Management

The package version is defined in two places:

- `gymnasium_jsbsim/__init__.py` - `__version__` variable
- `pyproject.toml` - `version` field

Keep these synchronized when releasing new versions.

## Making Changes

1. Create a new branch for your feature/fix
2. Make your changes
3. Run tests and quality checks
4. Commit your changes (pre-commit hooks will run automatically)
5. Push and create a pull request

## Troubleshooting

### Pre-commit hooks fail

If pre-commit hooks fail, fix the issues and try committing again:

```bash
# Let the tools auto-fix what they can
black gymnasium_jsbsim/
isort --profile black gymnasium_jsbsim/
ruff check --fix gymnasium_jsbsim/

# Then commit again
git add .
git commit -m "Your message"
```

### Tests fail

Ensure JSBSim is properly installed:

```bash
python -c "import jsbsim; print(jsbsim.__version__)"
```

### Import errors

Reinstall the package in editable mode:

```bash
pip install -e .
```
