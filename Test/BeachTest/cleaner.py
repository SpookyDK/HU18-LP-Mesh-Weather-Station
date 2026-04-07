import re
import sys

ansi_escape = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


def clean(filepath: str):
    with open(filepath, "r") as file:
        data = file.readlines()

    output = []
    reaced_start = False
    for line in data:
        if not reaced_start:
            if "TXtask: Sent Packet with id: '0'" in line:
                reaced_start = True
            else:
                continue
        skip = False
        for trash in ["PCNT", "GPS", "bounce"]:
            if trash in line:
                skip = True
                continue
        if skip:
            continue
        output.append(ansi_escape.sub("", line))

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
