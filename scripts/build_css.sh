#!/bin/sh
# Regenerates src/html/styles.css from the classes used in src/html/*.html.
# Needs node/npm; installs tailwindcss and daisyui into scripts/css/node_modules.
set -e
cd "$(dirname "$0")/css"
if [ ! -d node_modules/daisyui ]; then
    npm install --no-save --silent tailwindcss@3.4.17 daisyui@4.7.3
fi
npx tailwindcss -c tailwind.config.js -i input.css -o ../../src/html/styles.css --minify
