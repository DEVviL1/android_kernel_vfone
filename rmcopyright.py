#!/usr/bin/python
import os 
import sys
from commands import *
import re
print "===> Searching Contain 'MediaTek' file ... ..."
outputfilelist = getoutput('grep -nr --color --exclude=".git" "MediaTek" .').split('\n')
newfilelist = []
for line in outputfilelist:
 newline = line.split(':')[0]
newfilelist.append(newline)
newfilelist = list(set(newfilelist))
print "===> Searching Done!"
for line in newfilelist:
    print "===> Operate file ...%s... start" %line
    if not os.path.exists(line):
        continue
fileR = open(line, 'r')
writeflag = 0
filecontent = []
for subline in fileR.readlines():
    if subline.startswith('/*') and subline.find('*/') == -1:
        writeflag = 1
    print "======> ignore line %s" %subline
    continue
    if subline.startswith('# Copyright Statement:'):
        writeflag = 2
    print "======> ignore line %s" %subline
    continue
    if writeflag == 1 and subline.find('*/') >= 0:
        writeflag = 0
    print "======> ignore line %s" %subline
    continue

if writeflag == 2 and subline.strip().startswith('#') and subline.strip().endswith('\\'):
  writeflag = 3 

if writeflag == 3 and not subline.strip().endswith('\\'):
  writeflag = 2
