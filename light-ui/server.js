// Smart Lighting web UI — serves the floor map, relays MQTT light/person
// updates over Socket.IO, and forwards calibration / master-control actions.
// Connection settings come from the project .env (see .env.example).

require('dotenv').config({ path: require('path').join(__dirname, '..', '.env') });

const express = require('express');
const http = require('http');
const socketio = require('socket.io');
const mqtt = require('mqtt');

const MQTT_HOST = process.env.MQTT_HOST || 'localhost';
const MQTT_PORT = process.env.MQTT_PORT || '1883';
const UI_PORT = parseInt(process.env.UI_PORT || '3000', 10);

const app = express();
const server = http.createServer(app);
const io = socketio(server);

app.use(express.json());

app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html');
});

const mqttClient = mqtt.connect(`mqtt://${MQTT_HOST}:${MQTT_PORT}`);

mqttClient.on('connect', () => {
  console.log(`Connected to MQTT ${MQTT_HOST}:${MQTT_PORT}`);
  mqttClient.subscribe('lights/#');
  mqttClient.subscribe('persons/positions');
});

mqttClient.on('message', (topic, payload) => {
  if (topic === 'persons/positions') {
    try {
      io.emit('person_update', JSON.parse(payload.toString()));
    } catch (e) {
      console.error('persons/positions parse error:', e.message);
    }
    return;
  }

  const parts = topic.split('/');
  const row = parseInt(parts[1]);
  const col = parseInt(parts[2]);
  const state = payload.toString() === 'ON';
  io.emit('light_update', { row, col, state });
});

let floorClicks = [];

app.post('/click', (req, res) => {
  const { tile_col, tile_row, label, cam } = req.body;
  floorClicks.push({ tile_col, tile_row, label, cam });
  console.log(`floor click: cam=${cam} label=${label} tile=(${tile_col},${tile_row})`);
  res.json({ ok: true });
});

app.get('/clicks', (req, res) => {
  res.json(floorClicks);
});

app.delete('/clicks', (req, res) => {
  floorClicks = [];
  res.json({ ok: true });
});

// drag-to-correct calibration: forward the correction to Python over MQTT
app.post('/calibrate', (req, res) => {
  const { cam, foot_x, foot_y, tile_col, tile_row } = req.body;
  mqttClient.publish('calibration/add', JSON.stringify({ cam, foot_x, foot_y, tile_col, tile_row }));
  console.log(`calibrate: ${cam} foot(${foot_x},${foot_y}) -> tile(${tile_col},${tile_row})`);
  res.json({ ok: true });
});

app.post('/calibrate/undo', (req, res) => {
  const { cam } = req.body;
  mqttClient.publish('calibration/undo', JSON.stringify({ cam }));
  console.log(`calibrate undo: ${cam}`);
  res.json({ ok: true });
});

// master real-light control: ALL ON / ALL OFF / AUTO
app.post('/lights/all', (req, res) => {
  const { mode } = req.body;   // "on" | "off" | "auto"
  const cmd = mode === 'on' ? 'ALL_ON' : mode === 'off' ? 'ALL_OFF' : 'AUTO';
  mqttClient.publish('control/lights', cmd);
  console.log(`real lights master: ${cmd}`);
  res.json({ ok: true });
});

server.listen(UI_PORT, () => {
  console.log(`Open http://localhost:${UI_PORT}`);
});
