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
//const uid = fs.readFileSync('login.txt', 'utf8');  -------- PRODUCTION
const uid = 'ry9WLV5Rz81Al7wcRusTy12vlHAk9VrX'; //   -------- TESTING


//connection request
socket.on('connect', function(){
    socket.emit('connectclover', uid);
    socket.on('connectres'+uid, (user)=> {
        if(user!= null){
            console.log('Connected to server');
            let telemetrystreaminterval = setInterval(telemetryStream, 42);
            function telemetryStream(){
                if(getTelemetry.ros.isConnected){
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        if(telemetry.armed!=null && telemetry.z!=null && telemetry.pitch!=null && telemetry.roll!=null && telemetry.yaw!=null && telemetry.cell_voltage!=null){
                            socket.emit('telemetry', {uid: uid, armed: telemetry.armed, z: telemetry.z, lat: telemetry.lat, lon: telemetry.lon, alt: telemetry.alt, pitch: telemetry.pitch, roll: telemetry.roll, yaw: telemetry.yaw, cell_voltage: telemetry.cell_voltage.toFixed(2)})
                        }
                    });
                }
                else{
                    if(restartCounter != 10){
                        restartCounter++;
                    }
                    else{
                        exec("sudo pm2 restart /var/www/CRTClover/server.js -i max", (error, stdout, stderr) => {});
                        restartCounter = 0;
                    }
                }
            }

            socket.on('command', (command)=>{
                if(command.command == 'land'){
                    land.callService(new ROSLIB.ServiceRequest({}), function(result) {});
                }
                else if(command.command == 'hover'){
                    navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body' }), function(result) {});
                }
                else if(command.command == 'disarm'){
                    exec("python3 /var/www/CRTClover/disarm.py", (error, stdout, stderr) => {});
                }
                else if(command.command == 'photo'){
                    if (fs.existsSync('./photo.png')){
                        fs.unlinkSync(__dirname+'/photo.png');
                    }
                    exec("python3 /var/www/CRTClover/photo.py", (error, stdout, stderr) => {
                        if(!error){
                            let pfc = new Buffer(fs.readFileSync(__dirname+'/photo.png')).toString('base64');
                            socket.emit('photo', {uid: uid, photo: pfc});
                            fs.unlinkSync(__dirname+'/photo.png');
                        }
                    });
                }
                else if(command.command == 'rth'){
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        let arming = false;
                        let data = command.data;
                        if(!telemetry.armed){arming = true}
                        if(data.to == 'user'){
                            navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: data.alt-telemetry, yaw: 0.0, yaw_rate: 0.0, speed: 1, frame_id: 'body', auto_arm: arming }), function(result) {});
                            setTimeout(returnTO, (data.alt-telemetry)*1000+8000);
                            function returnTO(){
                                navigate_global.callService(new ROSLIB.ServiceRequest({ lat: data.lat, lon: data.lon, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: data.speed, frame_id: 'body', auto_arm: false }), function(result) {
                                });
                                let x = data.lat - telemetry.lat;
                                let y = data.lon - telemetry.lon;
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
                        else if(data.to == 'takeoff'){
                            //TODO
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