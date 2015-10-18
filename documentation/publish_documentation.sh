#!/bin/bash -e

# Quit on errors.
set -o errexit -o nounset

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
git config user.name "${COMMIT_USER}"
git config user.email "${COMMIT_EMAIL}"

cd ${HTML_PATH}

git init
git remote add upstream "https://$PA_TOKEN@github.com/${COMMIT_USER}/berkeley_sfm.git"
git fetch upstream
git reset upstream/gh-pages

touch .
git add -A .
git commit -m "Automated documentation build for changeset ${CHANGESET}."
git push -q upstream HEAD:gh-pages



# Make sure branches are up to date.
#git remote set-branches --add origin gh-pages
#git fetch origin

## Remove stale documentation.
#git checkout -b gh-pages origin/gh-pages
#if [ -d "${HTML_PATH}" ]; then
#  # Delete all documentation and push.
#  cd ${HTML_PATH}
#  git rm -rf ./*
#  cd ..
#  git commit -a -m "Automated documentation for changeset ${CHANGESET}: step 1 of 2 -  Deleting old documentation."
#  git push origin gh-pages
#fi
#echo "-- Successfully removed stale documentation."

## Make the documentation from source files on the master branch.
#git checkout master
#if [ -d "${BUILD_PATH}" ]; then
#  cd ${BUILD_PATH}
#  make documentation
#  cd -

#  # Checkout the gh-pages branch, transferring all documentation.
#  git checkout gh-pages

#  # Publish the documentation.
#  cd ${HTML_PATH}
#  git add .
#  git commit -m "Automated documentation for changeset ${CHANGESET}: step 2 of 2 - Adding new documentation."
#  git push origin gh-pages
#  cd -
#fi
#echo "-- Successfully updated documentation!"
