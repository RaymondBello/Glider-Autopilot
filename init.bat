start putty -load AVA

cd D:\GITHUB\Glider-Autopilot\Ground Control Software\Python Telemetry Dashboard\server
start npm start 
cd ..

SLEEP 5

start cmd.exe /C python "GCS_PFD.py"

start cmd.exe /C python "GCS_PLOT.py" t