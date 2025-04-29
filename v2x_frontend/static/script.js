// Establish a connection to the Socket.IO server
const socket = io();

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

  // Follow GPS
  // map.panTo([data.latitude, data.longitude], {
  //   animate: true,
  //   duration: 0.2
  // });
});

// Emit a custom event to the server
function sendMessageToServer(message) {
  socket.emit('message', message);
}


// leaflet map;
var map = L.map('map').setView([47.6644544, 9.491444], 13);
// Add a tile layer to the map
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
}).addTo(map);


// Add Ego vehicle marker
var egoVehicle = L.circle([47.6644544, 9.491444], { color: 'red', radius: 8, fillOpacity: 0.5 }).addTo(map);
// var egoVehicle = L.marker([47.6644544, 9.491444]).addTo(map);


var intersections = {}

socket.on('lane_paths', (data) => {
  data = JSON.parse(data);
  console.log('Parsed lane path data:', data);

  // data = {
  //   "id": intersection.id, 
  //   "refPoint": intersection.refPoint,
  //   "lanes": []
  //   }

  if (intersections[data.id]) {
    return;
  }

  intersections[data.id] = [];

  // Add intersection number
  // let pop = L.popup()
  //   .setLatLng([data.refPoint.lat / 10000000, data.refPoint.lon / 10000000])
  //   .setContent(data.id)
  //   .openOn(map);



  for (let i = 0; i < data.lanes.length; i++) {
    intersections[data.id].push(L.polyline(data.lanes[i], { color: 'blue', radius: 5, fillOpacity: 0.5 }).addTo(map));
  }

}
)