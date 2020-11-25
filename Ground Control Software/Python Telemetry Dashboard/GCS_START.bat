cd server 
start npm start 
cd ..

SLEEP 1

start cmd.exe /C python "GCS_PFD.py"

start cmd.exe /C python "GCS_PLOT.py" t