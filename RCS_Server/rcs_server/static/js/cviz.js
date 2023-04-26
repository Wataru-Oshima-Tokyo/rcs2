bootstrap_alert = function (message) {
  $('#alert_holder').html('<div class="alert alert-primary fade show" style="width:30%; margin: auto;" role="alert"><a class="close" data-dismiss="alert">×</a><span>' + message + '</span></div>');
}
bootstrap_warn = function (message) {
  $('#alert_holder').html('<div class="alert alert-warning fade show" style="width:30%; margin: auto;" role="alert"><a class="close" data-dismiss="alert">×</a><span>' + message + '</span></div>');
}

show_error_message = function (message) {
  $('#alert_holder').html('<div class="alert alert-warning fade show" style="width:30%; margin: auto; font-size: 24px;" role="alert"><a class="close" data-dismiss="alert">×</a><span>' + message + '</span></div>');
}

show_successful_message = function (message) {
  $('#alert_holder').html('<div class="alert alert-primary fade show" style="width:30%; margin: auto; font-size: 24px;" role="alert"><a class="close" data-dismiss="alert">×</a><span>' + message + '</span></div>');
}


var canvas = document.getElementById("myCanvas");
var cameraCanvas = document.getElementById("camera_canvas");
var cameraCtx = cameraCanvas.getContext("2d");
var stage = new createjs.Stage("myCanvas");
var map_scale = 1;
var robots = [];
var mode = 0
var targe_color = "#FFC0CB"
const modeArray = ["OBSERVATION", "INITPOSE", "NAVIGATION","WAYPOINT", "CONTROL", "WP_DISTRIBUTOR"];
const actionArray = ["waypoint", "charge", "doghouse"];

let x1 = 0;
let y1 = 0;
let x2 = 0;
let y2 = 0;
let symbol;

var init_pose = new createjs.Container();
var goal_pose = new createjs.Container();
var waypoint_max = 0;
var waypoint_counter = 0;
var waypointArray = [];

function createSymbol(r, x, y) {
  var g = new createjs.Graphics().beginFill("#00f0ff").drawCircle(0, 0, r);
  var s = new createjs.Shape(g);
  s.x = x;
  s.y = y;
  return s;
}

function createBaseMessage(type) {
  var message = {
    msgtype: type,
    sender: 'RCS',
    duration: 120,
    command_id: 0
  };
  return message;
}

function sendNav(message) {
  if (selected_robot.socket.readyState !== WebSocket.CLOSED) {
    selected_robot.socket.send(JSON.stringify({
      "navigation": message
    }));
    bootstrap_alert("Sent navigation message");
  } else {
    bootstrap_warn("Unable to send navigation message. Websocket connection not established.")
  }
}

function sendWaypoints(message){
  if (selected_robot.socket.readyState !== WebSocket.CLOSED) {
    selected_robot.socket.send(JSON.stringify({
      "waypoints": message
    }));
    bootstrap_alert("Sent waypoints message");
  } else {
    bootstrap_warn("Unable to send navigation message. Websocket connection not established.")
  }
}



function createCircle(r, x, y) {
  var myGraphics = new createjs.Graphics();
  var fillCommand = myGraphics.beginFill("blue").command;
  myGraphics.setStrokeStyle(1);
  myGraphics.beginStroke("#000000");
  myGraphics.drawCircle(0, 0, r);
  var s = new createjs.Shape();
  s.graphics = myGraphics;
  s.x = x;
  s.y = y;
  return [s, fillCommand];
}

function createTriangle(r) {
  var s = new createjs.Shape();
  var fillCommand = s.graphics.beginFill("#00ff00").command;
  s.graphics.moveTo(0, 0);
  s.graphics.lineTo(-r, r / 2);
  s.graphics.lineTo(0, -r * 2);
  s.graphics.lineTo(r, r / 2);
  s.graphics.closePath();
  s.y = r / 2;
  return [s, fillCommand];
}

function createRobot(r, x, y, name) {
  let container = new createjs.Container();
  let text = new createjs.Text(name, "20px Arial", "#ff0000");
  text.textBaseline = "alphabetic";
  text.x = -6 * text.text.length;
  text.y = -20;
  [circle, circle_fill] = createTriangle(7);
  circle.name = "icon";
  container.addChild(circle, text);
  container.x = x;
  container.y = y;
  stage.addChild(container);
  return [container, circle_fill];
}

function loadMap(mapname) {
  var mapimg = new Image();
  mapimg.src = "/static/images/" + mapname + ".png"
  var bmp = new createjs.Bitmap(mapimg);
  bmp.name = "mapimg";
  bmp.scale = map_scale;
  stage.addChild(bmp); // Might not show immediately
  stage.update();
}

function showDebug(x1, y1, degree, radians, length) {
  document.querySelector('#debugStatus').innerHTML = (
    "cooridnate:\n" +
    "x: " + x1 + "\n" +
    "y: " + y1 + "\n" +
    "theta(degree): " + degree + "\n" +
    "theta(radians): " + radians + "\n" +
    "length: " + length + "\n"
  );
}


function handle_pose() {
  let point = new createjs.Container();
  if (modeArray[mode] == "INITPOSE") {
    var action = new createjs.Text("init_pose", "20px Arial", "#800080");
    var id = new createjs.Text("", "20px Arial", "#FFFF00");
  } else if (modeArray[mode] == "NAVIGATION") {
    var action = new createjs.Text("goal_pose", "20px Arial", "#FF2400");
    var id = new createjs.Text("", "20px Arial", "#FFFF00");
  }
  else {
    var id = new createjs.Text(waypoint_max.toString(), "20px Arial", "#0000FF");
    var action = new createjs.Text("waypoint", "20px Arial", "#2ecc71");
  }
  action.textBaseline = "alphabetic";
  action.x = -6 * action.text.length;
  action.y = -20;
  id.textBaseline = "alphabetic";
  id.x = -6 * id.text.length;
  id.y = -40;
  [triangle, triangle_fill] = createTriangle(10);
  triangle.name = "icon";
  radians = Math.atan2((x2 - x1), (y1 - y2));
  var robot_radians = Math.atan2((y1 - y2), (x2 - x1)); //this is different from map radians
  triangle.rotation = radians * 180 / Math.PI;
  point.addChild(triangle, action, id);
  point.x = x1;
  point.y = y1;

  if (modeArray[mode] == "INITPOSE") {
    stage.removeChild(init_pose);
    init_pose = point;
    stage.addChild(init_pose);
    handle_navigation();

  } else if (modeArray[mode] == "WAYPOINT" || modeArray[mode] == "WP_DISTRIBUTOR") {
    let dict = { 'waypoint': point, 'color': triangle_fill, 'action': action, 'id': id, 'angle': robot_radians, 'action_id': 0 };
    waypointArray.push(dict);
    stage.addChild(waypointArray[waypoint_max]['waypoint']);
    waypointArray[waypoint_counter]['color'].style = targe_color;
    waypoint_max += 1;

  } else if (modeArray[mode] == "NAVIGATION") {
    stage.removeChild(goal_pose);
    triangle_fill.style = "#40E0D0";
    goal_pose = point;
    stage.addChild(goal_pose);
    handle_navigation();
  }
  stage.update();
  showDebug(x1, y1, triangle.rotation, radians, waypoint_counter);
}

function handle_navigation() {
  var mapim = stage.getChildByName("mapimg");
  robot_radians = Math.atan2((y1 - y2), (x2 - x1));
  let nav_massage = createBaseMessage(modeArray[mode]);
  nav_massage["x"] = x1;
  nav_massage["y"] = mapim.image.height - y1;
  nav_massage["th"] = robot_radians;
  nav_massage["action"] = "waypoint";
  sendNav(nav_massage);
  showDebug(x1, y1, radians * 180 / Math.PI, robot_radians, 0);
}

function handle_waypoint() {
  var mapim = stage.getChildByName("mapimg");
  let waypoint = waypointArray[waypoint_counter]['waypoint'];
  let robot_radians = waypointArray[waypoint_counter]['angle'];
  let action = waypointArray[waypoint_counter]['action'].text;
  let nav_massage = createBaseMessage("WAYPOINT");
  nav_massage["x"] = waypoint.x;
  nav_massage["y"] = mapim.image.height - waypoint.y;
  nav_massage["th"] = robot_radians;
  nav_massage["action"] = action;
  showDebug(nav_massage["x"], nav_massage["y"], nav_massage["th"], nav_massage["action"], 0);
  sendNav(nav_massage);
}

function handleCircularReferences() {
  const seenObjects = new Set();

  return (key, value) => {
    if (typeof value === 'object' && value !== null) {
      if (seenObjects.has(value)) {
        // Detected a circular reference; replace it with a custom value or null
        return "[Circular]";
      }
      seenObjects.add(value);
    }
    return value;
  };
}

function handle_waypoint_distributor(){
  var mapim = stage.getChildByName("mapimg");
  let waypoints_massage = createBaseMessage("WP_DISTRIBUTOR");
  var robot_waypoints = [];
  waypoints_massage["map_id"] = localeName;
  waypoints_massage["map_height"] = mapim.image.height;
  waypoints_massage["map_width"] = mapim.image.width;
  for (var i = 0; i < waypoint_max; i++) {
    let waypoint ={};
    waypoint["x"] = waypointArray[i]['waypoint'].x;
    waypoint["y"] = mapim.image.height - waypointArray[i]['waypoint'].y;
    waypoint["th"] = waypointArray[i]['angle'];
    robot_waypoints.push(waypoint);
  }
  const jsonString = JSON.stringify(robot_waypoints, handleCircularReferences());
  waypoints_massage["waypoints"] = jsonString;
  showDebug(waypoints_massage["map_id"],  waypoints_massage["map_height"], waypoints_massage["map_width"], waypoints_massage["waypoints"] , 0);
  sendWaypoints(waypoints_massage);
}

function convertCoordinatesMapToImg(realx, realy, resolution, originx, originy) {
  let pixelx = (realx - originx) / resolution;
  let pixely = (realy - originy) / resolution;
  return [pixelx, pixely];
}

function handle_keyExplanation() {
  if (modeArray[mode] == "OBSERVATION") {
    document.querySelector('#Explanation').innerHTML = (
      "     c: MODE CHANGE\n" +
      "You can move the map by dragging the mouse."
    );
  }
  else if (modeArray[mode] == "NAVIGATION") {
    document.querySelector('#Explanation').innerHTML = (
      "     c: MODE CHANGE\n" +
      "You can set the goal by doing mousedown and set the direction as you dragging like rviz"
    );
  }
  else if (modeArray[mode] == "INITPOSE") {
    document.querySelector('#Explanation').innerHTML = (
      "     c: MODE CHANGE\n" +
      "You can set the init pose by doing mousedown and set the direction as you dragging like rviz"
    );
  }
  else if (modeArray[mode] == "WAYPOINT") {
    document.querySelector('#Explanation').innerHTML = (
      "     c: MODE CHANGE\n" +
      "     i: WAYPOINT TARGET UP\n" +
      "     m: WAYPOINT TARGET DOWN\n" +
      "     d: REMOVE THE TARGET WAYPOINT\n" +
      "     s: START WAYPOINT NAVIGATION\n" +
      "     a: CHANGE THE ACTION OF THE TARGET WAYPOINT\n" +
      "You can add a waypoint by doing mousedown and set the direction as you dragging like rviz"
    );
  }
  else if (modeArray[mode] == "CONTROL") {
    document.querySelector('#Explanation').innerHTML = (
      "     c: MODE CHANGE\n"
    );
  }

}


function init() {
  console.log("Window loaded.")
  loadMap(localeName);
  // Zoom and Pan
  var mode_type = document.getElementById("mode");
  mode_type.innerHTML = modeArray[mode];
  addEventListener("keydown", handleKeydown);
  handle_keyExplanation();
  var zoom;
  function MouseWheelHandler(e) {
    if (Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail))) > 0)
      zoom = 1.1;
    else
      zoom = 1 / 1.1;
    var local = stage.globalToLocal(stage.mouseX, stage.mouseY);
    stage.regX = local.x;
    stage.regY = local.y;
    stage.x = stage.mouseX;
    stage.y = stage.mouseY;
    stage.scaleX = stage.scaleY *= zoom;
    // Lock scaling at certain points
    if (stage.scaleX > 1.3) {
      stage.scaleX = 1.3;
      stage.scaleY = 1.3;
    } else if (stage.scaleX < 0.9) {
      stage.scaleX = 0.9;
      stage.scaleY = 0.9;
    }
    stage.update();
  }

  function handleKeydown(event) {
    if (event.keyCode == KEYCODE.c) {
      if (mode >= modeArray.length - 2) {
        mode = 0;
      } else {
        mode += 1;
      }
      var mode_type = document.getElementById("mode");
      mode_type.innerHTML = modeArray[mode];
      handle_keyExplanation();
      //Change the target waypoint --------------
    } else if (event.keyCode == KEYCODE.d) {
      stage.removeChild(waypointArray[waypoint_counter]['waypoint']);
      waypointArray.splice(waypoint_counter, 1);
      waypoint_max -= 1;
      // Update their name and id
      for (var i = waypoint_counter; i < waypoint_max; i++) {
        waypointArray[i]['action'].text = waypointArray[i]['action'].text;
        waypointArray[i]['id'].text = i;
      }
      if (waypoint_counter > 0)
        waypoint_counter -= 1;
      waypointArray[waypoint_counter]['color'].style = targe_color;
      stage.update();
    }
    //change the waypoint taraget up
    else if (event.keyCode == KEYCODE.i) {
      if (waypoint_counter < waypoint_max - 1) {
        waypoint_counter += 1;
      }
      //update the color
      if (waypoint_counter > 0) {
        waypointArray[waypoint_counter - 1]['color'].style = "#00ff00";
      }
      waypointArray[waypoint_counter]['color'].style = targe_color;
    }
    //change the waypoint taraget down
    else if (event.keyCode == KEYCODE.m) {
      if (waypoint_counter > 0) {
        waypoint_counter -= 1;
      }
      //update the color
      if (waypoint_counter < waypoint_max - 1) {
        waypointArray[waypoint_counter + 1]['color'].style = "#00ff00";
      }
      waypointArray[waypoint_counter]['color'].style = targe_color;
    }
    //start waypoint navigation 
    else if (event.keyCode == KEYCODE.s) {
      if (modeArray[mode] == "WAYPOINT") {
        if (waypoint_max > 0) {
          handle_waypoint();
        }
      }else if (modeArray[mode] == "WP_DISTRIBUTOR"){
        if (waypoint_max > 0) {
          handle_waypoint_distributor();
        }
      }
    } else if (event.keyCode == KEYCODE.e && modeArray[mode] == "WAYPOINT") {
        mode = modeArray.length - 1;
        mode_type = document.getElementById("mode");
        mode_type.innerHTML = modeArray[mode];
        handle_keyExplanation();
    }
    //change the target waypoint action
    else if (event.keyCode == KEYCODE.a) {
      if (modeArray[mode] == "WAYPOINT" || modeArray[mode] == "WP_DISTRIBUTOR") {
        if (waypoint_max > 0) {
          if (waypointArray[waypoint_counter]['action_id'] < actionArray.length - 1) {
            waypointArray[waypoint_counter]['action_id'] += 1;
          } else {
            waypointArray[waypoint_counter]['action_id'] = 0;
          }
          waypointArray[waypoint_counter]['action'].text = actionArray[waypointArray[waypoint_counter]['action_id']];
        }
      }
    }
  }

  //--------------------
  stage.on("stagemousedown", function (e) {
    var offset = { x: stage.x - e.stageX, y: stage.y - e.stageY };
    x1 = e.stageX - stage.x;
    y1 = e.stageY - stage.y;
    symbol = createSymbol(7, x1, y1);
    stage.addChild(symbol);
    stage.update();
    if (modeArray[mode] == "OBSERVATION") {
      stage.on("stagemousemove", function (ev) {
        stage.x = ev.stageX + offset.x;
        stage.y = ev.stageY + offset.y;
        stage.update();
      });
    }
  });

  //---------------------
  stage.on("stagemouseup", function (ev) {
    stage.removeChild(symbol);
    if (modeArray[mode] == "OBSERVATION") {
      stage.removeAllEventListeners("stagemousemove");
    }
    else if (modeArray[mode] == "INITPOSE" || modeArray[mode] == "WAYPOINT" || modeArray[mode] == "WP_DISTRIBUTOR" || modeArray[mode] == "NAVIGATION") {
      x2 = ev.stageX - stage.x;
      y2 = ev.stageY - stage.y;
      handle_pose();
    }
  });
}
// End Zoom and Pan
createjs.Ticker.on("tick", stage); // Update at stable interval


window.onload = init();