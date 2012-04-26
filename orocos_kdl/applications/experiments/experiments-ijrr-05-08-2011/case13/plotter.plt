set term post enh color "Helvetica" 8
set output 'plot.eps'



set size 1,1
set origin 0,0
set grid 
set key on
set key left bottom
set xzeroaxis
set multiplot



set origin 0,0
set xlabel "time (sec)"
set ylabel "E (m)"
plot "tempCartDataCase13-Euler-CLIK-Error-halfxgain" using 1:2 with lines  title "Ey-PD-with-0.5xgain", \
"tempCartDataCase13-Euler-CLIK-Error-halfxgain" using 1:3 with lines  title "Ey-PD-with-0.5xgain", \
"tempCartDataCase13-Euler-CLIK-Error-halfxgain-PID" using 1:2 with lines  title "Ex-PID-with-0.5xgain",\
"tempCartDataCase13-Euler-CLIK-Error-halfxgain-PID" using 1:3 with lines  title "Ey-PID-with-0.5xgain"


unset multiplot
set size 1,1
set origin 0,0
set multiplot


set origin 0,0
set xlabel "time (sec)"
set ylabel "E (m)"
plot "tempCartDataCase13-Euler-CLIK-Error-halfxgain" using 1:2 with lines  title "Ey-PD-with-0.5xgain", \
"tempCartDataCase13-Euler-CLIK-Error-halfxgain" using 1:3 with lines  title "Ey-PD-with-0.5xgain"


unset multiplot
set size 1,1
set origin 0,0
set multiplot


set origin 0,0
set xlabel "time (sec)"
set ylabel "E (m)"
plot "tempCartDataCase13-Euler-CLIK-Error-halfxgain-PID" using 1:2 with lines  title "Ex-PID-with-0.5xgain",\
"tempCartDataCase13-Euler-CLIK-Error-halfxgain-PID" using 1:3 with lines  title "Ey-PID-with-0.5xgain"



unset multiplot
set size 1,1
set origin 0,0
set multiplot


set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian poses (m)"
plot "tempCartDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:2 with lines  title "x-no-cont-no-const", \
"tempCartDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:3 with lines title "y-no-cont-no-const", \
"tempCartDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:2 with lines  title "x-no-cont-with-const", \
"tempCartDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:3 with lines title "y-no-cont-with-const", \
"tempCartDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:2 with lines  title "x-with-cont-with-const", \
"tempCartDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:3 with lines title "y-with-cont-with-const"

unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:6 with lines  title "ax-no-cont-no-const",\
"tempCartDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:7 with lines title "ay-no-cont-no-const",\
"tempCartDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:6 with lines  title "ax-no-cont-with-const",\
"tempCartDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:7 with lines title "ay-no-cont-with-const",\
"tempCartDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:6 with lines  title "ax-with-cont-with-const",\
"tempCartDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:7 with lines title "ay-with-cont-with-const"



unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint pose (rad)"
plot "tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:2 with lines  title "j0-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:3 with lines title "j1-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:2 with lines title "j0-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:3 with lines title "j1-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:2 with lines title "j0-with-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:3 with lines title "j1-with-cont-with-const"


unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint acc (rad/s*s)"
plot "tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:6 with lines  title "j0-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:7 with lines title "j1-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:6 with lines title "j0-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:7 with lines title "j1-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:6 with lines title "j0-with-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:7 with lines title "j1-with-cont-with-const"

unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "torque (Nm)"
plot "tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:8 with lines  title "j0-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-no-constraint" using 1:9 with lines title "j1-no-cont-no-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:8 with lines title "j0-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-no-control-with-constraint" using 1:9 with lines title "j1-no-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:8 with lines title "j0-with-cont-with-const",\
"tempJointDataCase13-Euler-CLIK-Actual-with-control-with-constraint" using 1:9 with lines title "j1-with-cont-with-const