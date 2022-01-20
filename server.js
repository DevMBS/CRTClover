//import libs
const { exec } = require("child_process");
const ROSLIB = require('roslib');
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
const getTelemetry = new ROSLIB.Service({ ros: ros, name : '/get_telemetry', serviceType : 'clover/GetTelemetry' });
const navigate = new ROSLIB.Service({ ros: ros, name : '/navigate', serviceType : 'clover/Navigate' });
const land = new ROSLIB.Service({ ros: ros, name : '/land', serviceType : 'clover/Land' });
const navigate_global = new ROSLIB.Service({ ros: ros, name : '/navigate_global', serviceType : 'clover/NavigateGlobal' });
const request = require("request");
const fs = require('fs');
const { MongoClient } = require('mongodb');
const url = 'https://abbe-95-73-121-127.eu.ngrok.io/';
//get login and password of user
const logindata = fs.readFileSync('login.txt', 'utf8').split(';');
//connection request
request.post(
    url+'connect',
    { json: { login: logindata[0], password: logindata[1]} },
    function (error, response, body) {
        if(response.body.uid == null || response.body.db == null){
            console.log('Error! Please Re-install the app.');
        }
        else{
            //get uid from server response
            const uid = String(response.body.uid);
            const uri = String(response.body.db);
            const client = new MongoClient(uri, { useNewUrlParser: true, useUnifiedTopology: true });
            //connecting to command database client
            client.connect(err => {
                if (err) {
                    console.log('Connection error: ', err);
                    throw err;
                }
                else{
                    console.log('Connected to CRTCloversideCommandDB');
                }
                //connecting to command db
                const db = client.db('CRTCloversideCommandDB');
                const user = db.collection(logindata[0]);
                function insertOne(collection, data){
                    try {
                        collection.insertOne(data);
                    } catch (error) {
                        console.log(error);
                    }
                }
                //starting telemetry stream
                let telemetrystreaminterval = setInterval(telemetrystream, 1000);
                //telemetry stream
                function telemetrystream(){
                    getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                        if(telemetry.armed!=null && telemetry.z!=null && telemetry.pitch!=null && telemetry.roll!=null && telemetry.yaw!=null && telemetry.cell_voltage!=null){
                        insertOne(user, {data: 'telemetry', telemetry: {armed: telemetry.armed, z: telemetry.z, lat: telemetry.lat, lon: telemetry.lon, alt: telemetry.alt, pitch: telemetry.pitch, roll: telemetry.roll, yaw: telemetry.yaw, cell_voltage: telemetry.cell_voltage.toFixed(2)}});
                    }
                    });
                }
                getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telem) {
                    let checkEmergencyInterval = setInterval(checkEmergency, 100);
                    let x = telem.vx;
                    let y = telem.vy;
                    let z = telem.vz;
                    function checkEmergency(){
                        getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                            if(Math.abs(telemetry.vx - x) > 1 || Math.abs(telemetry.vy - y) > 1 || Math.abs(telemetry.vz - z) > 1){
                                exec("sudo systemctl restart clover", (error, stdout, stderr) => {});
                            }
                        });
                    }
                });
                //handle commands
                user.watch().on("change", next => {
                    if(next.operationType == 'insert' && next.fullDocument.data == 'command'){
                        if(next.fullDocument.command == 'land'){
                            land.callService(new ROSLIB.ServiceRequest({}), function(result) {
                            });
                        }
                        else if(next.fullDocument.command == 'hover'){
                            navigate.callService(new ROSLIB.ServiceRequest({ x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 0.1, frame_id: 'body' }), function(result) {});
                        }
                        else if(next.fullDocument.command == 'reboot'){
                            try {
                                exec("python emergency.py", (error, stdout, stderr) => {});
                            } catch (error) {}
                            setTimeout(function(){exec("sudo systemctl restart clover", (error, stdout, stderr) => {});}, 200);
                        }
                        else if(next.fullDocument.command == 'rth'){
                            getTelemetry.callService(new ROSLIB.ServiceRequest({ frame_id: '' }), function(telemetry) {
                                let arming = false;
                                let data = next.fullDocument.gps;
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
                                else if(next.fullDocument.gps.to == 'takeoff'){
                                    //TODO
                                }
                            });
                        }
                        else if(next.fullDocument.command == 'photo'){
                            if (fs.existsSync('./photo.png')){
                                fs.unlinkSync(__dirname+'/photo.png');
                            }
                            exec("python photo.py", (error, stdout, stderr) => {
                                if(!err){
                                    user.deleteMany({command: 'photo'});
                                    let pfc = new Buffer(fs.readFileSync(__dirname+'/photo.png')).toString('base64');
                                    insertOne(user, {data: 'photo', photo: pfc});
                                    fs.unlinkSync(__dirname+'/photo.png');
                                }
                            });
                        }
                    }
                });
            });
        }
});