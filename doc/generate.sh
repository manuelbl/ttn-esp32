#!/bin/sh
rm -rf html ttn-esp32.zip
doxygen Doxyfile
cd html
zip -r ../ttn-esp32.zip .