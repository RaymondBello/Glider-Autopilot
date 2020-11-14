const WebSocket = require("ws");
var fs = require('fs');
var rawData = fs.readFileSync('../sensor_data.json');
var JSONdata = JSON.parse(rawData);

const wss = new WebSocket.Server({ port: 8082 });


wss.on("connection", ws => {
    console.log("New Client Connected...");

    ws.on("message", data => {

        try {
            var data_ = JSON.parse(data);
            saveToJSON(data_);
            
        } catch (e) {
            console.log(e.message);
        }
        
        ws.send("[INFO] Data received by localhost");
    });

    ws.on("close", () => {
        console.log("Client Has disconnected.");
    })
});

/**
 * Saves JSON data to a local file.
 *
 * @param {JSON} data - Raw data to be saved into JSON file
 */
function saveToJSON(data) {
    console.log("[RECEIVED]", data);

    fs.writeFile('../trajectory.json', JSON.stringify(data), finished);

    function finished(err) {
        console.log('Wrote JSON file');
    };
    
}

console.log(JSONdata);