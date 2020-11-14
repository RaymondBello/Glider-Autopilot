// Table DATA
var waypoints = [
  {
    id: 0,
    pathName: "RAY>LEE",
    origin: [45.4041285310935, -75.71102868152072],
    destination: [45.414029442859935, -75.67247276512195],
    data: [
      [45.4041285310935, -75.71102868152072],
      [45.414029442859935, -75.67247276512195],
    ],
    climbRate: 4,
    cruiseALT: 70,
    cruiseSPD: 15,
  },
  {
    id: 1,
    pathName: "YND>YOW",
    origin: [45.521389, -75.564167],
    destination: [45.3225, -75.669167],
    data: [
      [45.521389, -75.564167],
      [45.3225, -75.669167],
    ],
    climbRate: 2,
    cruiseALT: 82,
    cruiseSPD: 11.5,
  },
  {
    id: 2,
    pathName: "YOW>YYZ",
    origin: [45.3225, -75.669167],
    destination: [43.676667, -79.630556],
    data: [
      [45.3225, -75.669167],
      [43.676667, -79.630556],
    ],
    climbRate: 3,
    cruiseALT: 50,
    cruiseSPD: 20,
  },
];

const ws = new WebSocket("ws://localhost:8082");

ws.addEventListener("open", (e) => {
  console.log("[INFO] Connected to localhost");

  ws.send("[INFO] Ground Control Station (active)");
});

ws.addEventListener("message", ({data}) => {
  console.log(data);
});

var waypointStack = new Array();
var selectedWaypoint = 0;

function createTable() {
  var tableData = document.getElementById("myTable");
  var rows = document.getElementById("myTable").rows;

  for (var i = 0; i < waypoints.length; i++) {
    var row = tableData.insertRow(i + 1);
    row.id = i + 1;
    row.setAttribute("onclick", "mapFunction(this.id);");
    row.insertCell(0).innerHTML = (i < 10 ? "A0" : "A") + waypoints[i].id;
    row.insertCell(1).innerHTML = waypoints[i].pathName;
    row.insertCell(2).innerHTML = waypoints[i].origin.toString();
    row.insertCell(3).innerHTML = waypoints[i].destination.toString();
    row.insertCell(4).innerHTML = waypoints[i].climbRate.toString();
    row.insertCell(5).innerHTML = waypoints[i].cruiseALT;
    row.insertCell(6).innerHTML = waypoints[i].cruiseSPD;
    waypoints[i].distance = calcDistancePoints(waypoints[i].data).toFixed(0);
    row.insertCell(7).innerHTML = waypoints[i].distance;
    waypoints[i].duration = getFlightDuration(waypoints[i]);
    row.insertCell(8).innerHTML = waypoints[i].duration;

    waypointStack.push(waypoints[i]);
  }
  console.log("Current Waypoint stack",waypointStack);
}

function getFlightDuration(obj) {
  return obj.id * obj.id + 2 * obj.id - obj.id + 2;
}

createTable();

var mymap = L.map("mapid").setView([45.41, -75.7], 13);

// "https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}"
// "pk.eyJ1IjoicmF5YmVsbCIsImEiOiJja2hjazhyd2wwN2l1MnN0M3RtdGd2aTRoIn0.4mpZFa8_T8XfLagyn7EsnQ"

L.tileLayer(
  "https://api.mapbox.com/styles/v1/raybell/ckhfmq5j30ad71anskil9i9tz/tiles/256/{z}/{x}/{y}@2x?access_token={accessToken}",
  {
    attribution:
      'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    maxZoom: 18,
    id: "mapbox/streets-v11",
    tileSize: 512,
    zoomOffset: -1,
    accessToken:
      "pk.eyJ1IjoicmF5YmVsbCIsImEiOiJja2hjazhyd2wwN2l1MnN0M3RtdGd2aTRoIn0.4mpZFa8_T8XfLagyn7EsnQ",
  }
).addTo(mymap);

var layerGroup = L.layerGroup().addTo(mymap);

var markerArray = new Array();

mymap.on("click", function (e) {
  var poplocation = e.latlng;
  var marker = L.marker([e.latlng.lat, e.latlng.lng]).addTo(layerGroup);
  // marker.addTo(layerGroup)
  var coordinates = [marker.getLatLng().lat, marker.getLatLng().lng];
  markerArray.push(coordinates);
  drawLine(markerArray);
  // console.log(markerArray);
  if (markerArray.length == 1) {
    marker.bindPopup("ORIGIN").openPopup();
  }
});

function mapFunction(id) {
  var rowData = document.getElementById("myTable").rows[id].cells;
  var rowObj = waypointStack[id - 1];

  // Debug Output
  // console.log(rowObj);

  // Clear Layers
  layerGroup.clearLayers();

  // Add to Map
  poly = drawLine(rowObj.data);
  mymap.closePopup();
  originMarker = L.marker(rowObj.origin).addTo(layerGroup);
  destinationMarker = L.marker(rowObj.destination).addTo(layerGroup);
  originMarker
    .bindPopup(
      "#" +
        rowData[0].innerHTML +
        " | " +
        "ORIGIN" +
        " | " +
        "Distance: " +
        rowObj.distance +
        "m" +
        " | " +
        "Points: " +
        rowObj.data.length
    )
    .openPopup();
  destinationMarker.bindPopup("DESTINATION");

  selectedWaypoint = rowObj.id
}

function calcDistance(origin, dest) {
  const R = 6371e3; // metres
  const φ1 = (origin[0] * Math.PI) / 180; // φ, λ in radians
  const φ2 = (dest[0] * Math.PI) / 180;
  const Δφ = ((dest[0] - origin[0]) * Math.PI) / 180;
  const Δλ = ((dest[1] - origin[1]) * Math.PI) / 180;
  const a =
    Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
    Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const d = R * c; // in metres
  return d;
}

function drawLine(marray) {
  var polyline = L.polyline(marray, { color: "red" }).addTo(mymap);
  polyline.addTo(layerGroup);

  if (marray.length > 2) {
    for (var i = 1; i < marray.length - 1; i++) {
      L.circle(marray[i], {
        color: "red",
        fillColor: "#f03",
        fillOpacity: 0.5,
        radius: 35,
      }).addTo(layerGroup);
    }
  }
}

function updateJSON() {

  var selectedPathObj = waypointStack[selectedWaypoint];

  if (ws.readyState == 0) {
    alert(
      "[ERROR] \nWebsocket not connected\nCannot send path: " + selectedPathObj.pathName
    );
  }
  if (ws.readyState == 1) {
    ws.send(JSON.stringify(selectedPathObj));
    alert(
      "[INFO] [SUCCESS]\nWebsocket connected\nSent path: " + selectedPathObj.pathName
    );
  }
  if (ws.readyState == 2) {
    alert("[ERROR] \nWebsocket closing\nCannot send path: " + selectedPathObj.pathName);
  }
  if (ws.readyState == 3) {
    alert("[ERROR] \nWebsocket Closed\nRe-launch server and try again.");
  }
}

function clearMarker() {
  layerGroup.clearLayers();
  markerArray = [];
}

function addToTable() {
  var table = document.getElementById("myTable");
  var rows = document.getElementById("myTable").rows;
  var row = table.insertRow(rows.length);
  row.id = rows.length - 1;
  row.setAttribute("onclick", "mapFunction(this.id);");

  var obj_ = {};
  obj_.id = row.id - 1;
  obj_.pathName =
    (rows.length - 2 < 10 ? "CST>00" : "CST>0") + (rows.length - 2);
  obj_.origin = markerArray[0];
  obj_.destination = markerArray[markerArray.length - 1];
  obj_.data = markerArray;
  obj_.climbRate = 0;
  obj_.cruiseALT = 0;
  obj_.cruiseSPD = 0;
  obj_.distance = calcDistancePoints(obj_.data).toFixed(0);
  obj_.duration = getFlightDuration(obj_);

  row.insertCell(0).innerHTML =
    (rows.length - 2 < 10 ? "A0" : "A") + (rows.length - 2);
  row.insertCell(1).innerHTML = obj_.pathName;
  row.insertCell(2).innerHTML = obj_.origin;
  row.insertCell(3).innerHTML = obj_.destination;
  row.insertCell(4).innerHTML = obj_.climbRate;
  row.insertCell(5).innerHTML = obj_.cruiseALT;
  row.insertCell(6).innerHTML = obj_.cruiseSPD;
  row.insertCell(7).innerHTML = obj_.distance;
  row.insertCell(8).innerHTML = obj_.duration;

  waypointStack.push(obj_);
  markerArray = [];

  console.log(waypointStack);
}

function calcDistancePoints(narray) {
  var totalDistance = 0;
  for (var i = 0; i < narray.length - 1; i++) {
    totalDistance = totalDistance + calcDistance(narray[i], narray[i + 1]);
  }
  return totalDistance;
}

function deleteRow() {
  var rows = document.getElementById("myTable").rows;
  document.getElementById("myTable").deleteRow(rows.length - 1);
  waypointStack.pop();
}
