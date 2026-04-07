import re
import sys

ansi_escape = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


def clean(filepath: str):
    with open(filepath, "r") as file:
        data = file.readlines()

    output = []
    reaced_start = False
    cords = ""
    ids = ""
    for line in data:
        if not reaced_start:
            if "TXtask: Sent Packet with id: '0'" in line:
                reaced_start = True
            else:
                continue

        if "GPS: Latitude:Longitude" in line:
            cords = line.split("GPS: Latitude:Longitude:  ")[1][:-1]
            cords = cords.replace(" ", "")

        skip = False
        for trash in ["PCNT", "GPS", "bounce"]:
            if trash in line:
                skip = True
                continue
        if skip:
            continue
        line = ansi_escape.sub("", line)
        line = line[3:]
        line = line.replace(") ", ",")
        time = line.split(",")[0]
        if "Sent" in line:
            ids = re.findall(r"id: '(\d+)'", line)[0]
            line = ",".join([time, "sent", ids, cords]) + "\n"
        if "Bouncing" in line:
            ids = re.findall(r"id='(\d+)'", line)[0]
            line = ",".join([time, "bounced", ids, cords]) + "\n"
        if "Ignoring" in line:
            ids = re.findall(r"id='(\d+)'", line)[0]
            line = ",".join([time, "ignored", ids, cords]) + "\n"
        if "Unknown" in line:
            line = ",".join([time, "unknown", ids, cords]) + "\n"

        output.append(line)

    with open("clean_" + filepath, "w") as file:
        file.writelines(output)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Missing argument")
        print("The argument is the file(s) to clean")
        exit(1)

    for filepath in sys.argv[1:]:
        clean(filepath)
    print("Done.")
