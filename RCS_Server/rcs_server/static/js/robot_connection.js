class ConnectedRobot {
  constructor(name) {
    this.lastMessageRecieved = -1;
    // Setup Socket and its functions
    this.socket = new WebSocket(
      'ws://'
      + window.location.host
      + '/ws/dispatch/user/'
      + name
      + '/'
    );
    // MUST BIND THIS FOR HIGH ORDER FUNCTIONS
    // https://eliux.github.io/javascript/common-errors/why-this-gets-undefined-inside-methods-in-javascript/
    this.socket.onopen = this._Open.bind(this);
    this.socket.onmessage = this._Message.bind(this);
    this.socket.onclose = this._Close.bind(this);
    this.socket.onerror = this._Error.bind(this);
    this.name = name;
    [this.stage_object, this.circleFill] = createRobot(5, 0, 0, name);
    this.isActive = false;
    setInterval(this._CheckConnection.bind(this), 1000);
  }

  _CheckConnection() {
    console.log("Robot: " + this.name + " Checking connection!");
    if (!this.isActive) return;
    var ConStatusDom = document.querySelector('#ConStat');
    if (Math.floor(Date.now() / 1000) - this.lastMessageRecieved > 5) {
      ConStatusDom.innerHTML = ('Status: <span class="badge bg-danger">Not connected!</span> ');
    } else {
      ConStatusDom.innerHTML = ('Status: <span class="badge bg-success">Connected!</span> ');
    }
  }

  // Perform On open  
  _Open(e) {
    var ConStatusDom = document.querySelector('#ConStat');
    ConStatusDom.innerHTML = ('Status: <span class="badge bg-success">Connected!</span> ');
  }

  // Perform on message recieve
  _Message(e) {
    this.lastMessageRecieved = Math.floor(Date.now() / 1000);
    const data = JSON.parse(e.data);
    var msgtype = data.message.msgtype;
    if (msgtype == "TELEMETRY") {
      // let x = data.message.x;
      // let y = data.message.y;
      let pixelCoordinates = convertCoordinatesMapToImg(data.message.x, data.message.y, data.message.resolution, data.message.origin_x, data.message.origin_y);
      let x = pixelCoordinates[0];
      let y = pixelCoordinates[1];
      let theta = data.message.theta;
      let battery_level = data.message.battery_level
      console.log("Recieved message from " + this.name + ":" + x + " " + y + " " + theta);
      let mapim = stage.getChildByName("mapimg");
      this.stage_object.x = x;
      this.stage_object.y = mapim.image.height - y;
      let theta_sym = (Math.PI - 1.5708) - theta; //convert to map angle

      this.stage_object.getChildByName("icon").rotation = theta_sym * 180 / Math.PI;

      // Battery status
      if (battery_level >= 80) {
        this.circleFill.style = "green";
      } else if (battery_level < 80 && battery_level >= 40) {
        this.circleFill.style = "yellow";
      } else {
        this.circleFill.style = "red";
      }

      if (this.isActive) {
        document.querySelector('#telemetryIn').innerHTML = (
          'x: ' + JSON.stringify(this.stage_object.y) + '\n' +
          'y: ' + JSON.stringify(this.stage_object.y) + '\n' +
          'theta(robot): ' + JSON.stringify(data.message.theta) + '\n' +
          'status: ' + JSON.stringify(data.message.status) + '\n' +
          'battery_level: ' + JSON.stringify(data.message.battery_level) + '\n'
        );
        // If message has a an image tag, Display image in camera canvas. 
        if (data.message.hasOwnProperty("img")) {
          var image = new Image();
          image.src = "data:image/jpeg;base64," + data.message.img;
          image.onload = function () {
            cameraCtx.drawImage(image, 0, 0);
          };
        }
      }
      console.log("update");
      stage.update();
    }
    else if (msgtype == "NAVIGATION_RESPONSE") {
      if (this.isActive) {
        var result = data.message.result;
        document.querySelector('#debugStatus').innerHTML = (
          'result ' + JSON.stringify(result) + '\n' +
          'result ' + waypoint_counter + '\n'
        );
      }
      if ((modeArray[mode] == "WAYPOINT" || modeArray[mode] == "OBSERVATION")&& JSON.stringify(result) == "true") {
        if (waypoint_counter < waypoint_max - 1) {
          waypoint_counter += 1;
          waypointArray[waypoint_counter - 1]['color'].style = "#00ff00";
          waypointArray[waypoint_counter]['color'].style = targe_color;
          handle_waypoint();
        }
      }
    }
    else if (msgtype == "COMMAND") {
      if (data.message.command == "ACTION_RESULT") {
        let param = JSON.parse(data.message.param);
        if (param.result == false) {
          show_error_message("The " + param.action.toUpperCase() + " ACTION" + " has FAILED...");
        }
        else {
          show_successful_message("The " + param.action.toUpperCase() + " ACTION" + " has SUCCEEDED!");
        }

      }

    }


  }

  // Perform On close
  _Close(e) {
    console.error('Robot socket ' + this.name + ' closed unexpectedly');
    bootstrap_warn("Could not establish connection to robot channel.")
    var ConStatusDom = document.querySelector('#ConStat');
    ConStatusDom.innerHTML = ('Status: <span class="badge bg-danger">Not connected!</span> ');
  }

  // Perform On error
  _Error(e) {
    console.error(this.name + " CLOSED: ERROR");
  }



}




robotNames = JSON.parse(document.getElementById('robot-names').textContent);
for (var i = 0; i < robotNames.length; i++) {
  c1 = new ConnectedRobot(robotNames[i]);
  robots.push(c1);
}
var selected_robot = null;

setActiveRobot = function (indx) {
  if (selected_robot != null)
    selected_robot.isActive = false;
  selected_robot = robots[indx];
  selected_robot.isActive = true;
  selected_robot._CheckConnection();
}
setActiveRobot(0); // Set inital robot active
// Set currently selected robot dropdown values
setRobotSelectors = function () {
  var dropdown = document.getElementById("selectedRobot");
  for (var i = 0; i < robots.length; i++) {
    var options = document.createElement("option");
    options.text = robots[i].name;
    options.value = i;
    dropdown.add(options);
  }
}
setRobotSelectors();

onrobotselectionchange = function () {
  var new_idx = document.getElementById('selectedRobot').value;
  setActiveRobot(new_idx);
}

document.querySelector('#customMessage').onkeyup = function (e) {
  if (e.keyCode === 13) {  // enter, returnfailure
    document.querySelector('#customMessageSend').click();
  }
};

document.querySelector('#customMessageSend').onclick = function (e) {
  console.log("sending");
  const messageInputDom = document.querySelector('#customMessage');
  const message = messageInputDom.value;
  messageInputDom.value = '';
  if (selected_robot.socket.readyState !== WebSocket.CLOSED) {
    selected_robot.socket.send(JSON.stringify({
      'message': message
    }));
    bootstrap_alert("Sent custom message: " + message);
  } else {
    bootstrap_warn("Unable to send custom message. Websocket connection not established.")
  }



};
