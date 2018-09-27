import re
import sys

#arg1:custom

print sys.argv
file_path = "config/param_" + sys.argv[1].lower() + ".h"

version_template = "#define BUILD_VER   %-5d\n"

f = open(file_path, "rw+")

original = f.read()

o=re.search(r'^#define\s+BUILD_VER\s+(\d+)', original, re.M)
ver_build=int(o.group(1))
i=o.start()

print ver_build

ver_build+=1

new_version = version_template % (ver_build)

print new_version

f.seek(i, 0)
f.writelines(new_version)

f.close()
