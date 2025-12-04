#!/bin/bash
# Sync Markdown documentation to GitHub Wiki
# Usage: ./sync-wiki.sh

set -e

REPO_URL="https://github.com/JGalego/gymnasium-jsbsim.wiki.git"
WIKI_DIR="wiki-temp"

echo "🔄 Syncing documentation to GitHub Wiki..."

# Clone the wiki repository
if [ -d "$WIKI_DIR" ]; then
    echo "📁 Wiki directory exists, pulling latest changes..."
    cd "$WIKI_DIR"
    git pull
    cd ..
else
    echo "📦 Cloning wiki repository..."
    git clone "$REPO_URL" "$WIKI_DIR"
fi

# Generate fresh markdown docs
echo "📝 Generating Markdown documentation..."
cd docs
make markdown
cd ..

# Copy markdown files to wiki
echo "📋 Copying documentation files..."
cp docs/markdown/index.md "$WIKI_DIR/Home.md"
cp docs/markdown/modules.md "$WIKI_DIR/API-Reference.md"

# Commit and push
cd "$WIKI_DIR"
if git diff --quiet && git diff --staged --quiet; then
    echo "✅ No changes to commit"
else
    echo "💾 Committing changes..."
    git add .
    git commit -m "Update documentation ($(date '+%Y-%m-%d %H:%M'))"
    echo "🚀 Pushing to GitHub Wiki..."
    git push
    echo "✅ Wiki updated successfully!"
fi

cd ..
echo "🎉 Done!"
