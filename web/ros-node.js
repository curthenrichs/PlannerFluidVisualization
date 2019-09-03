/*
MIT License

Copyright (c) 2019 Curt Henrichs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Web ROS Node

Implements ,mapping to/from visualization function API and ROS using the ROS
WebTools for ROS Bridge.

Subscribers:
  - /visualization/trajectory_start := VisualizationTrajectory
    "Used to start perturbing fluid"
  - /visualization/trajectory_update := VisualizationTrajectory
    "Updates perturbation within fluid"
  - /visualization/trajectory_stop := Empty
    "Stops new perturbation until started again"

Services Provided:
  - /visualization/set_config := SetConfig
    "Provides ability to set all configuration parameters within the
     visualization script. Note, used to set color."
  - /visualization/get_config := GetConfig
    "Provides access to read all configuration parameters within the
     visualization script"
*/

document.addEventListener('DOMContentLoaded', function(event) {

  var ros = new ROSLIB.Ros();

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  ros.connect('ws://localhost:9090');

  var trajectoryStartListener = new ROSLIB.Topic({
    ros: ros,
    name: '/visualization/trajectory_start',
    messageType: 'planner_fluid_visualization/VisualizationTrajectory'
  });
  trajectoryStartListener.subscribe(function(message) {
    console.log('Start');
    visualize_trajectoryStart(message.offsetX, message.offsetY);
  });

  var trajectoryUpdateListener = new ROSLIB.Topic({
    ros: ros,
    name: '/visualization/trajectory_update',
    messageType: 'planner_fluid_visualization/VisualizationTrajectory'
  });
  trajectoryUpdateListener.subscribe(function(message) {
    console.log('Update');
    visualize_trajectoryUpdate(message.offsetX, message.offsetY);
  });

  var trajectoryStopListener = new ROSLIB.Topic({
    ros: ros,
    name: '/visualization/trajectory_stop',
    messageType : 'std_msgs/Empty'
  });
  trajectoryStopListener.subscribe(function(message) {
    console.log('Stop');
    visualize_trajectoryStop();
  });

  var setConfigServer = new ROSLIB.Service({
    ros: ros,
    name: '/visualization/set_config',
    serviceType: 'planner_fluid_visualization/SetConfig'
  });
  setConfigServer.advertise(function(request, response) {
    try {
      visualize_setConfig(JSON.parse(request['data']));
      response['success'] = true;
    } catch (e) {
      response['success'] = false;
    }
    return true;
  });

  var getConfigServer = new ROSLIB.Service({
    ros: ros,
    name: '/visualization/get_config',
    serviceType: 'planner_fluid_visualization/GetConfig'
  });
  getConfigServer.advertise(function(request, response) {
    response['data'] = JSON.stringify(visualize_getConfig());
  });

});
