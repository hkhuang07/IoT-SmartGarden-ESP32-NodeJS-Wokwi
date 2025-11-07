const aedes = require("aedes")();
const net = require("net");
const { connectToDatabase } = require("./db_connection");
const { time } = require("console");

const MQTT_PORT = 1883;
let db;

//Define Data Handler (Xử lý Dữ liệu từ các thiết bị IoT gửi lên)
aedes.on("publish", async function (packet, client) {
  if (client) {
    const topic = packet.topic;
    const payload = packet.payload.toString();
    console.log(
      `Message from client ${client.id} on topic ${topic}: ${payload}`
    );

    if (topic.startsWith("smartgarden/sensor/")) {
      try {
        const data = JSON.parse(payload);
        const doc = {
          topic: topic,
          deviceId: client.id || "unknown",
          timestamp: new Date(),
          data: data,
        };

        await db.collection("sensor_readings").insertOne(doc);
        console.log(`[DB] Data saved from', ${doc.deviceId}`);
      } catch (error) {
        console.error(
          `[HANDLER ERROR] Processing error or DB save error: ${error.message}`
        );
      }
    }
    /* Logic cho Topic Điều khiển (smartgarden/control/) sẽ được Broker xử lý tự động
     *
     *
     *
     */
  }
});


async function startServer() {
    db = await connectToDatabase();
    const server = net.createServer(aedes.handle);
    server.listen(MQTT_PORT, function () {
      console.log(`MQTT broker started and listening on port ${MQTT_PORT}`);
    });
}

startServer();