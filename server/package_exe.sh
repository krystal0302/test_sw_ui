#!/bin/bash

echo "Gen version..."
bash ./get-version.sh
echo "Gen version finish..."

# CONFIG_FILE=pkg_config.json
CONFIG_FILE=pkg.config.json

packaging_app() {
	# TODO: arguments number protection
	# TODO: probe the pkg module exist or not
	# TODO: probe the config existence
	pkg $1 --options max-old-space-size=250 -c $CONFIG_FILE -C GZip -o $2
	return
}

echo "Packaging far-swarm-ui..."
packaging_app "app.js" "far-swarm-ui"

if [ ! -d "$PWD/bin" ]; then
	mkdir "$PWD/bin"
fi

cp -t ./bin far-swarm-ui email_conf.yaml settings.json farobottech.db version.json


