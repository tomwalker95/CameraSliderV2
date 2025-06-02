const express = require('express');
const http = require('http');
const path = require('path');
const mqtt = require('mqtt');
const socketIo = require('socket.io');

// MQTT connection parameters
targetHost = '192.168.50.9';
const MQTT_PORT     = 1883;
const TOPIC_PREFIX  = 'camera-slider';

// Connect via plain MQTT TCP
const mqttClient = mqtt.connect({
  host: targetHost,
  port: MQTT_PORT,
  protocol: 'mqtt'
});

const app    = express();
const server = http.createServer(app);
const io     = socketIo(server);

// Serve the static web page from './public'
app.use(express.static(path.join(__dirname, 'public')));

// MQTT event handlers
mqttClient.on('connect', () => {
  console.log('? MQTT connected to tcp://' + targetHost + ':' + MQTT_PORT);
});
mqttClient.on('error', err => {
  console.error('? MQTT error:', err.message);
});

// Handle WebSocket connections from the browser
io.on('connection', (socket) => {
  console.log('? Browser connected:', socket.id);

  socket.on('axis-command', ({axis, position}) => {
    if (typeof position !== 'number') return;
    const topic   = `${TOPIC_PREFIX}/axis/${axis}/command`;
    mqttClient.publish(topic, String(position));
    console.log(`? Published ${topic} = ${position}`);
  });

  socket.on('button-press', ({button}) => {
    const topic = `${TOPIC_PREFIX}/button/${button}/pressed`;
    mqttClient.publish(topic, '1');
    console.log(`? Published ${topic}`);
  });

  socket.on('disconnect', () => {
    console.log('? Browser disconnected:', socket.id);
  });
});

// Start the HTTP & WebSocket server
const PORT = 3000;
server.listen(PORT, () => {
  console.log(`? Server running at http://localhost:${PORT}`);
});
