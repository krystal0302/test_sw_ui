const process = require("process");
// NOTE:
//  PKG_INSTALL_DIR will be passed within the Makefile target context for local build.
//  Or, this value will set to default to align the current build convention.
const targetDir = process.env.PKG_INSTALL_DIR || "/home/farobot";

const Plugins = [
  {
    from: "../server/bin/",
    to: `${targetDir}/far_flm_ws/install/far_swarm_ui/`,
  },
];

module.exports = Plugins;
