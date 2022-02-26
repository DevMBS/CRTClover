#!/usr/bin/env node
//import libs an framework
//child_process - library for executing commands in terminal
const { exec } = require("child_process");
//roslib - js lib for ROS
const ROSLIB = require('roslib');
//connect ros to the 9090 port
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
//define some ros services
const getTelemetry = new ROSLIB.Service({ ros: ros, name : '/get_telemetry', serviceType : 'clover/GetTelemetry' });
const navigate = new ROSLIB.Service({ ros: ros, name : '/navigate', serviceType : 'clover/Navigate' });
const land = new ROSLIB.Service({ ros: ros, name : '/land', serviceType : 'clover/Land' });
const navigate_global = new ROSLIB.Service({ ros: ros, name : '/navigate_global', serviceType : 'clover/NavigateGlobal' });
//fs - file system library
const fs = require('fs');
//import socket.io clientside library (Clover will connect to the server as standard client)
const {io} = require('socket.io-client')
//connect sockets to the server
const socket = io.connect('https://48c5-94-29-124-254.eu.ngrok.io');
const url = 'https://48c5-94-29-124-254.eu.ngrok.io/';
//get uid of user
const uid = fs.readFileSync('/var/www/CRTClover/login.txt', 'utf8').split('\n')[0];

//delete last takeoff coordinates (if they exist)
if (fs.existsSync('/var/www/CRTClover/takeoff.txt')){
    fs.unlinkSync('/var/www/CRTClover/takeoff.txt');
}

//if sockets connected to the server
socket.on('connect', function(){
    //send connection request
    socket.emit('connectclover', uid);
    //handle connection response
    socket.on('connectres'+uid, (user)=> {
        //if user exist and Clover connected to the server
        if(user!= null){
            console.log('Connected to the server');
            //define telemetry stream interval (24 times per second)
            let telemetrystreaminterval = setInterval(telemetryStream, 42);
            //when the get_telemetry service is unavailable for some time and after it becomes available, the cloverside server still does not receive data from it, then it is solved only by restarting the server
            let restartCounter = 0;
            function telemetryStream(){
                //if get_telemetry is available
                if(getTelemetry.ros.isConnected){
                    //get data from the service
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        //check data
                        if(telemetry.armed!=null && telemetry.z!=null && telemetry.pitch!=null && telemetry.roll!=null && telemetry.yaw!=null && telemetry.cell_voltage!=null){
                            //send telemetry to the server
                            socket.emit('telemetry', {uid: uid, armed: telemetry.armed, z: telemetry.z, lat: telemetry.lat, lon: telemetry.lon, alt: telemetry.alt, pitch: telemetry.pitch, roll: telemetry.roll, yaw: telemetry.yaw, cell_voltage: telemetry.cell_voltage.toFixed(2)})
                            //if there is the first arming with gps, save takeoff coordinates
                            if (!fs.existsSync('/var/www/CRTClover/takeoff.txt') && telemetry.armed && telemetry.lat != null){
                                fs.writeFileSync('/var/www/CRTClover/takeoff.txt', `${telemetry.lat};${telemetry.lon}`);
                            }
                        }
                    });
                }
                else{
                    //if get_telemetry is unavailable, restart cloverside server ~2 times per second
                    if(restartCounter != 10){
                        restartCounter++;
                    }
                    else{
                        exec("pm2 restart /var/www/CRTClover/server.js");
                        restartCounter = 0;
                    }
                }
            }
            //handle commands from the server
            socket.on('command', (command)=>{
                //land request
                if(command.command == 'land'){
                    //stop drone, set mode to OFFBOARD with auto_arm (set_mode mavros function does not work well) and land
                    navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}), function(result) {
                        land.callService(new ROSLIB.ServiceRequest({}));
                    });
                }
                //hover request: set mode to OFFBOARD and stop the drone
                else if(command.command == 'hover'){
                    navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}));
                }
                //disarm request: disarm drone
                else if(command.command == 'disarm'){
                    exec("python3 /var/www/CRTClover/mavros.py disarm", (error, stdout, stderr) => {
                        if(stderr.indexOf('ModuleNotFoundError') >= 0){
                            exec("python /var/www/CRTClover/mavros.py disarm");
                        }
                    });
                }
                //photo request
                else if(command.command == 'photo'){
                    //delete last photo, if it exist
                    if (fs.existsSync(__dirname+'/photo.png')){
                        fs.unlinkSync(__dirname+'/photo.png');
                    }
                    //get photo
                    exec("python3 /var/www/CRTClover/mavros.py photo", (error, stdout, stderr) => {
                        if(!error){
                            //encode image to the base64 encoding
                            let pfc = new Buffer(fs.readFileSync(__dirname+'/photo.png')).toString('base64');
                            //send photo to the server as a base64 data
                            socket.emit('photo', {uid: uid, photo: pfc});
                            //delete image from the drone
                            fs.unlinkSync(__dirname+'/photo.png');
                        }
                        if(stderr.indexOf('ModuleNotFoundError') >= 0){
                            exec("python /var/www/CRTClover/mavros.py photo", (error, stdout, stderr) => {
                                if(!error){
                                    //encode image to the base64 encoding
                                    let pfc = new Buffer(fs.readFileSync(__dirname+'/photo.png')).toString('base64');
                                    //send photo to the server as a base64 data
                                    socket.emit('photo', {uid: uid, photo: pfc});
                                    //delete image from the drone
                                    fs.unlinkSync(__dirname+'/photo.png');
                                }
                            });
                        }
                    });
                }
                //return request
                else if(command.command == 'rth'){
                    let data = telemetry.data;
                    //get telemetry data
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        //if drone does not have gps data
                        if(telemetry.lat == null){
                            //send error to the server
                            socket.emit('rthError', {uid: uid});
                        }
                        else{
                            //the function can be executed
                            function rth(lat, lon){
                                //firstly, fly to specified altitude
                                navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: data.alt-telemetry.z, yaw: 0.0, yaw_rate: 0.0, speed: 0.5, frame_id: 'body', auto_arm: true }));
                                //set timeout for this function, alt/speed
                                setTimeout(returnTO, ((data.alt-telemetry.z)/0.5)*1000+4000);
                                //return function
                                function returnTO(){
                                    //fly to coordinates
                                    navigate_global.callService(new ROSLIB.ServiceRequest({ lat: lat, lon: lon, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: data.speed, frame_id: 'body', auto_arm: false }));
                                    let x = lat - telemetry.lat;
                                    let y = lon - telemetry.lon;
                                    //set time for this function (divide distance between coordinates by speed)
                                    setTimeout(action, ((Math.hypot(x, y)*1.113195e5)/data.speed)*1000);
                                    //hover or land after the return
                                    function action(){
                                        if(data.action == 'hover'){
                                            //hover
                                            navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: false }));
                                        }
                                        else if(data.action == 'land'){
                                            //land
                                            navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: false}), function(result) {
                                                land.callService(new ROSLIB.ServiceRequest({}));
                                            });
                                        }
                                    }
                                }
                            }
                            //if user choosed 'return to my coordinates'
                            if(data.to == 'user'){
                                rth(data.lat, data.lon);
                            }
                            //if user choosed 'return to takeoff coordinates'
                            else if(data.to == 'takeoff'){
                                //get takeoff coordinates
                                try {
                                    let coords = fs.readFileSync('/var/www/CRTClover/takeoff.txt', 'utf-8').split(';');
                                    let lat = coords[0];
                                    let lon = coords[1];
                                    rth(lat, lon);
                                } catch (error) {
                                    socket.emit('returnToTakeoffError', {uid: uid});
                                }
                            }
                        }
                    });
                }
            });
            //handle mission
            socket.on('mission', (mission) => {
                //write it to the python file
                fs.writeFileSync('/var/www/CRTClover/mission.py', mission);
                try {
                    //run mission
                    exec("python3 /var/www/CRTClover/mission.py", (error, stdout, stderr) => {
                        if(stderr.indexOf('ModuleNotFoundError') >= 0 && stderr.indexOf('rospkg') >= 0){
                            exec("python /var/www/CRTClover/mission.py", (error, stdout, stderr) => {
                                //send mission output
                                socket.emit('missionOut', {out: stdout, error: stderr, uid: uid});
                            });
                        }
                        else{
                            //send mission output
                            socket.emit('missionOut', {out: stdout, error: stderr, uid: uid});
                        }
                    });
                } catch (error) {}
            });
        }
        //error: the application was either installed incorrectly or there is no connection to the server
        else{
            console.log('Authentification failed. Please re-install the app or check internet connection.');
        }
    });
});