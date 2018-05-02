#!/usr/bin/python

import sys
from PIL import Image

fileName = sys.argv[1]

img = Image.open(fileName)
img = img.convert('L')

w, h = img.size
data = img.tobytes()

name = fileName[:fileName.rfind('.')]
name = name.replace(' ', '_')
line_buffer = []

print 'char %s_%dx%d[] = \\'%(name, w, h)
for p in data:
    line_buffer.append("\\x%02x"%ord(p))
    if len(line_buffer) >= 18:
        print '    "%s"\\'%(''.join(line_buffer))
        line_buffer = []
print ";"
