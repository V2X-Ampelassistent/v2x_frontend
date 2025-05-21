// Establish a connection to the Socket.IO server
const socket = io();

var follow_gps = false;

// Listen for the 'connect' event
socket.on('connect', () => {
  console.log('Connected to the server');
});

// Listen for the 'disconnect' event
socket.on('disconnect', () => {
  console.log('Disconnected from the server');
});


socket.on('message', (data) => {
  console.log('Message from server:', data);
});

socket.on('gps_data', (data) => {
  data = JSON.parse(data);
  console.log('Parsed GPS data:', data);

  egoVehicle.setLatLng([data.latitude, data.longitude]);
  direction = data.direction * Math.PI / 180;
  egoDirection.setLatLngs([[data.latitude, data.longitude], [data.latitude + 0.000075 * Math.cos(direction), data.longitude + 0.000075 * Math.sin(direction)]]);

  // reset current intersection id and lane id
  current_intersection_id = null;
  current_lane_id = null;

  // Follow GPS
  if (follow_gps) {
    map.panTo([data.latitude, data.longitude], {
      animate: true,
      duration: 0.2
    });
  }
});

// Emit a custom event to the server
function sendMessageToServer(message) {
  socket.emit('message', message);
}


// leaflet map;
var map = L.map('map').setView([47.6644544, 9.491444], 13);

// Add a tile layer to the map
osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
});

googleHybrid = L.tileLayer('http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}', {
  maxZoom: 20,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
});

// googleHybrid.addTo(map);
osm.addTo(map);

// Add Ego vehicle marker
var egoVehicle = L.circle([47.6644544, 9.491444], { color: 'red', radius: 4, fillOpacity: 0.5 }).addTo(map);
// var egoVehicle = L.marker([47.6644544, 9.491444]).addTo(map);

//////// V2X INTERSECTION ////////////

const STATE_LOOKUP = {
  "RED": "red",
  "YELLOW": "yellow",
  "GREEN": "green",
  "RED_YELLOW": "yellow",
  "STOP": "orange",
  "DARK": "black",
  "UNAVAILABLE": "gray",
}


var intersection_elements = {};
var intersection_popups = [];

socket.on('intersection', (intersection_data) => {
  intersection_data = JSON.parse(intersection_data);
  console.log('Parsed intersection data:', intersection_data);

  let id = intersection_data.id;
  let ref_point = intersection_data.ref_point;
  let lanes = intersection_data.lanes;

  if (intersection_elements[id]) {
    // Remove lanes
    for (let i = 0; i < intersection_elements[id].polyline.length; i++) {
      map.removeLayer(intersection_elements[id].polyline[i]);
    }
    for (let i = 0; i < intersection_elements[id].connections.length; i++) {
      map.removeLayer(intersection_elements[id].connections[i]);
    }
    intersection_elements[id].polyline = [];
    intersection_elements[id].connections = [];

  } else {
    // Create a popup for the new intersection
    let popup = L.popup(
      {
        autoClose: false,
        closeOnClick: false,
      })
      .setLatLng(ref_point)
      .setContent(`Intersection ID: ${intersection_id}`)
    // .openOn(map);

    intersection_popups.push(popup);

    // place to store the intersection elements
    intersection_elements[id] = {
      polyline: [],
      connections: [],
    }
  }

  // color = randomcolor();
  color = "#0000FF";

  // Create a polyline for the intersection
  for (let i = 0; i < lanes.paths.length; i++) {
    let lane_path = lanes.paths[i]

    // Create a polyline for the lane
    let lane_polyline = L.polyline(lane_path, { color: path_color, weight: 2 })
      .addTo(map)
      .on('click', function (e) {
        console.warn(`Lane ID: ${lane.id} on intersection: ${intersection_id}`);
        current_intersection_id = intersection_id;
        current_lane_id = lane.id;
      });
    intersection_elements[intersection_id].polyline.push(lane_polyline);
  }
  
  // Create a line for connecting the lanes
  for (let i = 0; i < lanes.connections.length; i++) {
    let lane_connection = lanes.connections[i]
    start = lane_connection.startpoint;
    end = lane_connection.endpoint;
    state = lane_connection.state;

    color = STATE_LOOKUP[state];
    if (color == undefined) {
      color = "black";
    }

    intersection_elements[id].connections.push(L.polyline([start, end], { color: color, weight: 2 }).addTo(map));
  }
});


socket.on('current_lane', (data) => {
  data = JSON.parse(data);
  current_intersection_id = data.intersection_id;
  current_lane_id = data.lane_id;
  console.log(`Current lane id: ${current_lane_id} on intersection: ${current_intersection_id} distance: ${data.distance}m`);
});

function randomcolor() {
  var letters = '0123456789ABCDEF';
  var color = '#';
  for (var i = 0; i < 6; i++) {
    color += letters[Math.floor(Math.random() * 16)];
  }
  return color;
}

let follow_gps_button = document.getElementById("follow_gps");
follow_gps_button.addEventListener("click", function () {
  follow_gps = !follow_gps;
  if (follow_gps) {
    follow_gps_button.innerHTML = "Stop following GPS";
  } else {
    follow_gps_button.innerHTML = "Follow GPS";
  }
});
