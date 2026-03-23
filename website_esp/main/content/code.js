function parsePayload(bytes) {
  const view = new DataView(bytes);
  let offset = 0;

  const data = {
    network_id: [view.getUint8(offset), (offset += 1)][0],
    orig_node_id: [view.getUint8(offset), (offset += 1)][0],
    packet_id: [view.getUint16(offset, true), (offset += 2)][0],
    hop_count: [view.getUint8(offset), (offset += 1)][0],
    flags: [view.getUint8(offset), (offset += 1)][0],
    longitude: [view.getInt32(offset, true) / 1e7, (offset += 4)][0],
    latitude: [view.getInt32(offset, true) / 1e7, (offset += 4)][0],
    air_humidity: [view.getUint8(offset), (offset += 1)][0],
    air_temp: [view.getInt16(offset, true) / 10.0, (offset += 2)][0],
    soil_temps: Array.from(
      { length: 4 },
      () => view.getInt16((offset += 2) - 2, true) / 16,
    ),
    soil_moisture: [view.getUint8(offset), (offset += 1)][0],
    pressure: [(view.getInt16(offset, true) + 100000) / 100, (offset += 2)][0],
    lux: [view.getUint16(offset, true), (offset += 2)][0],
    precipitation: [view.getUint16(offset, true) / 10, (offset += 2)][0],
    wind_speed: [view.getUint16(offset, true) / 1000, (offset += 2)][0],
  };
  return data;
}

function start_root() {
  setInterval(function () {
    fetch("/data").then((response) =>
      response.arrayBuffer().then((byte_array) => {
        for (const [key, value] of Object.entries(parsePayload(byte_array))) {
          if (key === "soil_temps") {
            document.getElementById(key).innerHTML = value
              .map((temp) => {
                return `<li>${temp} °C</li>`;
              })
              .join("");
          } else {
            document.getElementById(key).innerText = value;
          }
        }
      }),
    );
  }, 1000);
}

function start_viewer() {
  const gateway = `ws://${window.location.hostname}/dataviewer`;
  let websocket;
  let last_known_idx = 0;

  function initWebsocket() {
    console.log("Trying to open WebSocket connection...");
    websocket = new WebSocket(gateway);
    websocket.binaryType = "arraybuffer";

    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
  }
  function onOpen(event) {
    console.log("Connection opened");
    if (last_known_idx === 0) websocket.send("START_SD_STREAM");
    else websocket.send("START_SD_STREAM:" + last_known_idx);
  }
  function onClose(event) {
    console.log("Connection closed");
    setTimeout(initWebsocket, 2000);
  }
  function onMessage(event) {
    if (event.data instanceof ArrayBuffer) {
      handleData(event.data);
    } else {
      const target = "END_OF_TRANSMISSION";
      if (event.data.startsWith(target)) {
        last_known_idx = Number(event.data.split(":")[1]);
      }
      console.log("Received text: " + event.data);
    }
  }

  function handleData(buffer) {
    //TODO: The buffer contains more than one packet, interpret it
    const view = new DataView(buffer);
    const time = new Date(view.getInt32(0, true) * 1000);
    const count = view.getUint8(4);
    if (count < 1) {
      return;
    }
    let offset = 5;
    let stuff = `<li> ${time} <ul>`;
    for (let index = 0; index < count; index++) {
      const vals = parsePayload(buffer.slice(offset, offset + 34));
      stuff += `<li> ${vals["orig_node_id"]}`;
      stuff += `</li>`;
      offset += 34;
    }
    stuff += "</ul></li>";

    let elm = document.getElementById("my_list");
    elm.innerHTML = stuff + elm.innerHTML;
  }
  initWebsocket();
}
