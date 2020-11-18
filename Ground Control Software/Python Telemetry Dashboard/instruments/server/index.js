const WebSocket = require("ws");
var fs = require("fs");
var rawData = fs.readFileSync("../sensor_data.json");
var JSONdata = JSON.parse(rawData);

const wss = new WebSocket.Server({ port: 8082 });

wss.on("connection", (ws) => {
  console.log("\n[INFO] New Client Connected...");

  ws.on("message", (data) => {
    try {
      var data_ = JSON.parse(data);
      saveToJSON(data_);
    } catch (e) {
      console.log(data);
      console.log("[ERROR] Invalid JSON format. " + e.message);
    }

    ws.send("[INFO] Data received by localhost");
  });

  ws.on("close", () => {
    console.log("[INFO] Client Has disconnected.");
  });
});

/**
 * Saves JSON data to a local file.
 *
 * @param {JSON} data - Raw data to be saved into JSON file
 */
function saveToJSON(data) {
    console.log("[RECEIVED]", data);
    
    function finished(err) {
        console.log("[INFO] Wrote JSON file (../trajectory.json)");
    }

    fs.writeFile("../trajectory.json", JSON.stringify(data), finished);
}

console.log(JSONdata);
