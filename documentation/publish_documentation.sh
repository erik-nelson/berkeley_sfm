#!/bin/bash -e

# Settings.
HTML_PATH=documentation/
BUILD_PATH=build
COMMIT_USER="Automatic documentation builder"
CHANGESET=$(git rev-parse --verify HEAD)

# Hide email from crawlers.
COMMIT_EMAIL="eanelson"
COMMIT_EMAIL="${COMMIT_EMAIL}@eecs"
COMMIT_EMAIL="${COMMIT_EMAIL}.berkeley"
COMMIT_EMAIL="${COMMIT_EMAIL}.edu"

# Make sure branches are up to date.
git remote set-branches --add origin gh-pages
git fetch origin

# Remove stale documentation.
git checkout -b gh-pages origin/gh-pages
if [ -d "${HTML_PATH}" ]; then
  cd ${HTML_PATH}
  rm -rf ./*
  cd -
fi

# Make the documentation from source files on the master branch.
git checkout master
if [ -d "${BUILD_PATH}" ]; then
  cd ${BUILD_PATH}
  make documentation
  cd -

  # Checkout the documentation branch.
  git checkout gh-pages

  # Publish the documentation.
  cd ${HTML_PATH}
  git add .
  git config user.name "${COMMIT_USER}"
  git config user.email "${COMMIT_EMAIL}"
  git commit -m "Automated documentation build for changeset ${CHANGESET}."
  git push origin gh-pages
  cd -
fi
