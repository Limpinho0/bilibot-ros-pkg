#!/usr/bin/env python
import sys, urllib, base64
input_filename = sys.argv[1]
fin=open("identifier")
botwad="test"
botwad=fin.readline()[:-1]
fin.close()
passwad="uploadmyfile"
datawad = base64.encodestring(file("/tmp/toup", "rb").read())
postwad = urllib.urlencode({"pass":passwad,"bot":botwad,"filedata":datawad, "filename":input_filename})
file("/tmp/up.log", "wb").write(postwad)

