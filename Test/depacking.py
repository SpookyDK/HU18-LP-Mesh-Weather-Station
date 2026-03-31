import struct
import sys
from datetime import datetime

BE_PRETTY = False


def parse_sensor_data(file_path):
    # Format strings (little-endian '<')
    # B=uint8, H=uint16, i=int32, h=int16
    header_fmt = "<BBHBB"
    payload_fmt = "<iiBhhhhhBhHHH"

    # Combined format for one full_packet_t (31 bytes total)
    packet_fmt = header_fmt + payload_fmt[1:]  # Removing the '<' from payload_fmt
    packet_size = struct.calcsize(packet_fmt)

    packets = []

    try:
        with open(file_path, "rb") as f:
            while True:
                # Read the first byte to get the count
                time_data = f.read(4)
                if not time_data:
                    break

                count_data = f.read(1)
                if not count_data:
                    break

                packets = []
                time_stamp = datetime.fromtimestamp(struct.unpack("<i", time_data)[0])
                num_packets = struct.unpack("<B", count_data)[0]

                for _ in range(num_packets):
                    data = f.read(packet_size)
                    if len(data) < packet_size:
                        break

                    # Unpack the raw bytes
                    unpacked = struct.unpack(packet_fmt, data)

                    # Map unpacked data to a readable dictionary
                    packet = {
                        "header": {
                            "network_id": unpacked[0],
                            "orig_node_id": unpacked[1],
                            "packet_id": unpacked[2],
                            "hop_count": unpacked[3],
                            "flags": unpacked[4],
                        },
                        "payload": {
                            "longitude": unpacked[5] / 1e7,
                            "latitude": unpacked[6] / 1e7,
                            "air_humidity": unpacked[7],
                            "air_temperature": unpacked[8]
                            / 10.0,  # Assuming 1 decimal fixed-point
                            "soil_temperature": unpacked[9:13],  # The 4 soil sensors
                            "soil_moisture": unpacked[13],
                            "pressure": unpacked[14],
                            "spectrum": unpacked[15],
                            "precipitation": unpacked[16],
                            "wind_speed": unpacked[17],
                        },
                    }
                    packets.append(packet)
                yield packets, time_stamp

    except FileNotFoundError:
        print("File not found.")


# Usage
if __name__ == "__main__":
    counter = 0
    if len(sys.argv) != 2:
        print("Incorrect arg")
        exit(1)
    for arr, time_stamp in parse_sensor_data(sys.argv[1]):
        counter += 1
        if len(arr) == 1 and arr[0]["header"]["orig_node_id"] != 132 and not BE_PRETTY:
            continue

        if BE_PRETTY:
            print(f"Detected {len(arr)} packets in section from {time_stamp}")

        for p in arr:
            if p["header"]["orig_node_id"] == 132 and not BE_PRETTY:
                vals = [tuple(d.values()) for d in p.values()]
                vals = vals[0] + vals[1]
                vals = map(str, vals)
                print(str(time_stamp) + ";", end="")
                print(";".join(vals))

            if BE_PRETTY:
                print(
                    f"\tNode {p['header']['orig_node_id']}  Packet {p['header']['packet_id']} | {p['payload']['latitude']}, {p['payload']['longitude']},  {p['payload']['soil_temperature']}"
                )
        if BE_PRETTY:
            print()
    if BE_PRETTY:
        print("There was", counter, "packets")
