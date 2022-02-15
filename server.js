#!/usr/bin/env node
//import libs
const { exec } = require("child_process");
const ROSLIB = require('roslib');
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
const getTelemetry = new ROSLIB.Service({ ros: ros, name : '/get_telemetry', serviceType : 'clover/GetTelemetry' });
const navigate = new ROSLIB.Service({ ros: ros, name : '/navigate', serviceType : 'clover/Navigate' });
const land = new ROSLIB.Service({ ros: ros, name : '/land', serviceType : 'clover/Land' });
const navigate_global = new ROSLIB.Service({ ros: ros, name : '/navigate_global', serviceType : 'clover/NavigateGlobal' });
const fs = require('fs');
const {io} = require('socket.io-client')
const socket = io.connect('https://48c5-94-29-124-254.eu.ngrok.io');
const url = 'https://48c5-94-29-124-254.eu.ngrok.io/';
//get login and password of user
const uid = fs.readFileSync('/var/www/CRTClover/login.txt', 'utf8').split('\n')[0];

if (fs.existsSync('/var/www/CRTClover/takeoff.txt')){
    fs.unlinkSync('/var/www/CRTClover/takeoff.txt');
}

//connection request
socket.on('connect', function(){
    socket.emit('connectclover', uid);
    socket.on('connectres'+uid, (user)=> {
        if(user!= null){
            console.log('Connected to server');
            let telemetrystreaminterval = setInterval(telemetryStream, 42);
            let restartCounter = 0;
            function telemetryStream(){
                if(getTelemetry.ros.isConnected){
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        if(telemetry.armed!=null && telemetry.z!=null && telemetry.pitch!=null && telemetry.roll!=null && telemetry.yaw!=null && telemetry.cell_voltage!=null){
                            socket.emit('telemetry', {uid: uid, armed: telemetry.armed, z: telemetry.z, lat: telemetry.lat, lon: telemetry.lon, alt: telemetry.alt, pitch: telemetry.pitch, roll: telemetry.roll, yaw: telemetry.yaw, cell_voltage: telemetry.cell_voltage.toFixed(2)})
                            if (!fs.existsSync('/var/www/CRTClover/takeoff.txt') && telemetry.armed && telemetry.lat != null){
                                fs.writeFileSync('/var/www/CRTClover/takeoff.txt', `${telemetry.lat};${telemetry.lon}`);
                            }
                        }
                    });
                }
                else{
                    if(restartCounter != 10){
                        restartCounter++;
                    }
                    else{
                        exec("pm2 restart /var/www/CRTClover/server.js", (error, stdout, stderr) => {});
                        restartCounter = 0;
                    }
                }
            }

            socket.on('command', (command)=>{
                if(command.command == 'land'){
                    navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}), function(result) {
                        land.callService(new ROSLIB.ServiceRequest({}), function(result) {});
                    });
                }
                else if(command.command == 'hover'){
                    navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body', auto_arm: true}), function(result) {});
                }
                else if(command.command == 'disarm'){
                    exec("python3 /var/www/CRTClover/mavros.py disarm", (error, stdout, stderr) => {});
                }
                else if(command.command == 'photo'){
                    if (fs.existsSync(__dirname+'/photo.png')){
                        fs.unlinkSync(__dirname+'/photo.png');
                    }
                    exec("python3 /var/www/CRTClover/mavros.py photo", (error, stdout, stderr) => {
                        if(!error){
                            let pfc = new Buffer(fs.readFileSync(__dirname+'/photo.png')).toString('base64');
                            socket.emit('photo', {uid: uid, photo: pfc});
                            fs.unlinkSync(__dirname+'/photo.png');
                        }
                    });
                }
                else if(command.command == 'rth'){
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        if(telemetry.lat == null){
                            socket.emit('rthError', {uid: uid});
                        }
                        else{
                            function rth(lat, lon){
                                navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: data.alt-telemetry.z, yaw: 0.0, yaw_rate: 0.0, speed: 0.5, frame_id: 'body', auto_arm: true }), function(result) {});
                                setTimeout(returnTO, ((data.alt-telemetry.z)/0.5)*1000+4000);
                                function returnTO(){
                                    navigate_global.callService(new ROSLIB.ServiceRequest({ lat: lat, lon: lon, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: data.speed, frame_id: 'body', auto_arm: false }), function(result) {});
                                    let x = lat - telemetry.lat;
                                    let y = lon - telemetry.lon;
                                    setTimeout(action, (Math.sqrt(x*x+y*y)/data.speed)*1000+20000);
                                    function action(){
                                        if(data.action == 'hover'){
                                            navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 1, frame_id: 'body'}), function(result) {});
                                        }
                                        else if(data.action == 'land'){
                                            land.callService(new ROSLIB.ServiceRequest({}), function(result) {});
                                        }
                                    }
                                }
                            }
                            if(data.to == 'user'){
                                rth(data.lat, data.lon);
                            }
                            else if(data.to == 'takeoff'){
                                let lat = fs.readFileSync('/var/www/CRTClover/takeoff.txt', 'utf-8').split(';')[0];
                                let lon = fs.readFileSync('/var/www/CRTClover/takeoff.txt', 'utf-8').split(';')[0];
                                rth(lat, lon);
                            }
                        }
                    });
                }
            });

            socket.on('mission', (mission) => {
                fs.writeFileSync('/var/www/CRTClover/mission.py', mission);
                try {
                    exec("python3 /var/www/CRTClover/mission.py", (error, stdout, stderr) => {
                        socket.emit('missionOut', {out: stdout, error: error, uid: uid});
                    });
                } catch (error) {}
            });
        }
        else{
            console.log('Authentification failed. Please re-install the app.')
        }
    });
});