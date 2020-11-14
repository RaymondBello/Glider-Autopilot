const WebSocket = require("ws");

const wss = new WebSocket.Server({ port: 8082 });

wss.on("connection", ws => {
    console.log("New Client Connected...");

    ws.on("message", data => {
        console.log("[RECEIVED] " + data);
        
        ws.send("[INFO] Data received by localhost");
    });

    ws.on("close", () => {
        console.log("Client Has disconnected.");
    })
});
