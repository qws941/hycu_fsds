#!/bin/bash
set -e
cd "$(dirname "$0")/.."

rm -rf dist/submission dist/*.zip
mkdir -p dist/submission

cp -r src/simulator dist/submission/001_simulator
cp -r src/autonomous dist/submission/002_autonomous
cp docs/SUBMISSION_GUIDE.md dist/submission/README.md

cd dist
zip -r HYCU_FSDS_Submission.zip submission/
rm -rf submission
echo "Created: dist/HYCU_FSDS_Submission.zip"
ls -lh HYCU_FSDS_Submission.zip
