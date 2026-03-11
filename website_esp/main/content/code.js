function parsePayload(bytes) {
  const view = new DataView(bytes);
  let offset = 0;

  const data = {
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
    lux: [view.getInt16(offset, true), (offset += 2)][0],
    precipitation: [view.getUint16(offset, true) / 10, (offset += 2)][0],
    wind_speed: [view.getUint16(offset, true), (offset += 2)][0],
  };
  return data;
}

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
