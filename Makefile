UI_FOLDER=react_ui
SERVER_FOLDER=server
PKG_INSTALL_DIR= # default behavior: let the following build processes use its own value of `/home/farobot/`
.EXPORT_ALL_VARIABLES:

# install dependencies

init-pkg-install-dir:
	mkdir -p $(PKG_INSTALL_DIR)

install-system-deps:
	npm install --global pkg pm2

# TODO: use `npm ci -omit=dev`
install-ui-deps:
	cd $(UI_FOLDER) && npm ci

# TODO: use `npm ci -omit=dev`
install-server-deps:
	cd $(SERVER_FOLDER) && (rm -rf node_modules/ || true) && npm install

install-all: install-system-deps install-server-deps # install-ui-deps

# build artifacts

build-ui:
	cd $(UI_FOLDER) && npm run deploy

build-server:
	cd $(SERVER_FOLDER) && npm run dist

local-build-server: PKG_INSTALL_DIR=$(shell pwd)/local-build
local-build-server: init-pkg-install-dir build-server

local-build-all: local-build-server # build-ui

IMAGE_NAME=far_swarm_ui
BRANCH_NAME=$(shell git rev-parse --abbrev-ref HEAD)
COMMIT_SHORT_HASH=$(shell git rev-parse --short HEAD)
TIMESTAMP=$(shell date --utc +"%Y-%m-%dT%H-%M-%SZ")
local-build-image:
	docker build \
		-t $(IMAGE_NAME):$(BRANCH_NAME).latest \
		-t $(IMAGE_NAME):$(BRANCH_NAME).$(COMMIT_SHORT_HASH).$(TIMESTAMP) \
		-f ./Dockerfile .


# clean up

# clean-built-ui:
# 	(test -d $(UI_FOLDER)/build && rm -rf $(UI_FOLDER)/build) || true

# clean-deps-ui:
# 	(test -d $(UI_FOLDER)/node_modules/ && rm -rf $(UI_FOLDER)/node_modules/) || true

clean-built-server: PKG_INSTALL_DIR=$(shell pwd)/local-build
clean-built-server:
	(test -d $(PKG_INSTALL_DIR) && rm -rf $(PKG_INSTALL_DIR)) || true
	(test -d ./$(SERVER_FOLDER)/bin && rm -rf ./$(SERVER_FOLDER)/bin) || true

clean-deps-server:
	(test -d ./$(SERVER_FOLDER)/node_modules/ && rm -rf ./$(SERVER_FOLDER)/node_modules/) || true

clean-built: clean-built-server # clean-built-ui
clean-deps: clean-deps-server # clean-deps-ui
clean-all: clean-built clean-deps

#
# Targets for CI process
#

# NOTE: the sys. deps. are already installed on current build process
ci-install: install-server-deps install-ui-deps
ci-build: build-ui build-server
