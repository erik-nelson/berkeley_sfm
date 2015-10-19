#!/bin/bash -e

# Quit on errors.
set -o errexit -o nounset

# Settings.
DOCUMENTATION_PATH=documentation
CHANGESET=$(git rev-parse --verify HEAD)

# Set username and email. Hide email from crawlers.
COMMIT_USER="erik-nelson"
COMMIT_EMAIL="eanelson"
COMMIT_EMAIL="${COMMIT_EMAIL}@eecs.berkeley.edu"
git config user.name "${COMMIT_USER}"
git config user.email "${COMMIT_EMAIL}"

# Make sure branches are up to date.
git remote set-branches --add origin gh-pages
git fetch origin

# Commit new documentation from master branch.
touch ${DOCUMENTATION_PATH}
git add ${DOCUMENTATION_PATH}
git commit -m "."

# Check out gh-pages branch and merge documentation from master commit.
git checkout -b gh-pages origin/gh-pages
sudo rm -r ${DOCUMENTATION_PATH}
git checkout master ${DOCUMENTATION_PATH}

# Add the merged changes and push.
git add ${DOCUMENTATION_PATH}
git commit -m "Automated documentation build for changeset ${CHANGESET}."
git push -u origin gh-pages

echo "-- Successfully updated documentation!"
