import re
import pandas
import glob
parttern = re.compile(r"Received: b'(\d*\.\d*\.\d*) received (\d*) packages")


files = [x for x in glob.glob("*bytes 22.23.22 receiver.txt")]

for file in files:
    result = []
    with open(file, "r") as f:
        text_array = f.readlines()
        for line in text_array:
            if parttern.match(line) != None:
                setting, pdr = parttern.search(line).groups()
                print(setting, " ", pdr)
                result.append([setting, pdr])

    pandas.DataFrame(result).to_csv(f"{file[:-4]}.csv", header=["setting", "PDR"])