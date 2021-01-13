#!/bin/sh

## NOT WORKING YET

# Start Putty
# putty -load AVA
gnome-terminal --title "Serial Output" -- "putty " 

# Naviagate to Node Server Directory
gnome-terminal --title "Node Server" -- "cd Ground Control Software/Python Telemetry Dashboard/server; npm start"

sleep 3

gnome-terminal --title "PFD" -- "pwd"