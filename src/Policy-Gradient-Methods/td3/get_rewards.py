data = open("SomeFile (copy).txt").read()

import re

print(data)

matches = re.findall("Episode ([0-9]+): (.*)", data)

print(sorted([match[1] for match in matches]))

