data = open("SomeFile (copy).txt").read()

import re

matches = re.findall("Episode ([0-9]+): (.*)", data)

print(sorted(map(float, [match[1] for match in matches])))

