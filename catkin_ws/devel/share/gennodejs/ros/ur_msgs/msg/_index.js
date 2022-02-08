
"use strict";

let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let Digital = require('./Digital.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let Analog = require('./Analog.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let IOStates = require('./IOStates.js');

module.exports = {
  MasterboardDataMsg: MasterboardDataMsg,
  ToolDataMsg: ToolDataMsg,
  Digital: Digital,
  RobotModeDataMsg: RobotModeDataMsg,
  Analog: Analog,
  RobotStateRTMsg: RobotStateRTMsg,
  IOStates: IOStates,
};
