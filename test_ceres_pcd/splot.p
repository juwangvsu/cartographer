set term x11 0
splot 'sweep4.csv' using 1:2:4
set term x11 1
splot 'sweep4.csv' using 1:2:5
pause mouse "Click any mouse button on selected data point"
