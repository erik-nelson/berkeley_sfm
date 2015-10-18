#!/bin/bash -e

# Settings.
HTML_PATH=documentation/
BUILD_PATH=build/
CHANGESET=$(git rev-parse --verify HEAD)

# Set username and email. Hide email from crawlers.
COMMIT_USER="erik-nelson"
COMMIT_EMAIL="eanelson"
COMMIT_EMAIL="${COMMIT_EMAIL}@eecs"
COMMIT_EMAIL="${COMMIT_EMAIL}.berkeley"
COMMIT_EMAIL="${COMMIT_EMAIL}.edu"

# Make sure branches are up to date.
git remote set-branches --add origin gh-pages
git fetch origin

git config user.name "${COMMIT_USER}"
git config user.email "${COMMIT_EMAIL}"

# Remove stale documentation.
git checkout -b gh-pages origin/gh-pages
if [ -d "${HTML_PATH}" ]; then

  # Make sure travis has the ability to push.
  openssl aes-256-cbc -K $encrypted_6a2f0cd4845b_key -iv $encrypted_6a2f0cd4845b_iv -in ssh_keys/travisci_rsa.enc -out ssh_keys/travisci_rsa -d
  chmod 0600 ssh_keys/travisci_rsa
  cp ssh_keys/travisci_rsa ~/.ssh

  cd ${HTML_PATH}

  # Delete all documentation and push.
  git rm -rf ./*
  cd ..
  git add -A

  # Don't add the unencrypted ssh-key.
  git reset -- ssh_keys/travisci_rsa
  git commit -m "(1 of 2) Deleting documentation. Automated documentation build for changeset ${CHANGESET}."
  git push origin gh-pages
fi

# Make the documentation from source files on the master branch.
git checkout master
if [ -d "${BUILD_PATH}" ]; then
  cd ${BUILD_PATH}
  make documentation
  cd -

  # Checkout the gh-pages branch, transferring all documentation.
  git checkout gh-pages

  # Publish the documentation.
  cd ${HTML_PATH}
  git add .
  git commit -m "(2 of 2) Adding new documentation. Automated documentation build for changeset ${CHANGESET}."
  git push origin gh-pages
  cd -
fi
