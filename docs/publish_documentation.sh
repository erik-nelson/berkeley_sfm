#!/bin/bash -e

# Settings.
REPOSITORY_PATH=git@github.com:erik-nelson/berkeley_sfm.git
HTML_PATH=docs/html
BUILD_PATH=build
COMMIT_USER="Automatic documentation builder"
CHANGESET=$(git rev-parse --verify HEAD)

# Hide email from crawlers.
COMMIT_EMAIL="eanelson"
COMMIT_EMAIL="${COMMIT_EMAIL}@eecs"
COMMIT_EMAIL="${COMMIT_EMAIL}.berkeley"
COMMIT_EMAIL="${COMMIT_EMAIL}.edu"

# Make the documentation.
cd ${BUILD_PATH}
make documentation
cd -

# Publish the documentation.
cd ${HTML_PATH}
git add .
git config user.name "${COMMIT_USER}"
git config user.email "${COMMIT_EMAIL}"
git commit -m "Automated documentation build for changeset ${CHANGESET}."
git push origin master
cd -
