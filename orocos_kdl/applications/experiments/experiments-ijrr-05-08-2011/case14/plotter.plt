set term post enh color "Helvetica" 8
set output 'plot.eps'



set size 1,1
set origin 0,0
set grid 
set key on
set key left bottom
set xzeroaxis
set multiplot


#Joint error data ::no constraint case2 no stabilizer with PID

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint pose (rad)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:2 with lines  title "j0-e-no-const-with-joint-pidcontrol",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:3 with lines  title "j1-e-no-const-with-joint-pidcontrol"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint rate (rad/s)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:4 with lines  title "v0-e-no-const-with-joint-pidcontrol",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:5 with lines  title "v1-e-no-const-with-joint-pidcontrol"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint acc (rad/s*s)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:6 with lines  title "a0-e-no-const-with-joint-pidcontrol",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control-case2" using 1:7 with lines  title "a1-e-no-const-with-joint-pidcontrol"


#Cartesian actual data ::no constraint case no stabilizer with PID

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian pose (m)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:2 with lines  title "x-no-const-with-joint-pidcontrol",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:3 with lines  title "y-no-const-with-joint-pidcontrol"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian vel (m/s)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:4 with lines  title "vx-no-const-with-joint-pidcontrol",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:5 with lines  title "vy-no-const-with-joint-pidcontrol"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:6 with lines  title "ax-no-const-with-joint-pidcontrol",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control-case2" using 1:7 with lines  title "ay-no-const-with-joint-pidcontrol"




#Cartesian actual data ::with constraint case 2 no stabilizer with PID


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian pose (m)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:2 with lines  title "x-with-const-with-pidcontrol-no-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:3 with lines  title "y-with-const-with-pidcontrol-no-beta"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian vel (m/s)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:4 with lines  title "vx-with-const-with-pidcontrol-no-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:5 with lines  title "vy-with-const-with-pidcontrol-no-beta"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:6 with lines  title "ax-with-const-with-pidcontrol-no-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-no-stabilizer-PID-control-case2" using 1:7 with lines  title "ay-with-const-with-pidcontrol-no-beta"





#Cartesian actual data ::with constraint case 2 with stabilizer with PID

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian pose (m)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:2 with lines  title "x-with-const-with-pidcontrol-with-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:3 with lines  title "y-with-const-with-pidcontrol-with-beta"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian vel (m/s)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:4 with lines  title "vx-with-const-with-pidcontrol-with-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:5 with lines  title "vy-with-const-with-pidcontrol-with-beta"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:6 with lines  title "ax-with-const-with-pidcontrol-with-beta",\
"tempCartDataCase14-Euler-Actual-with-constraint-with-stabilizer-PID-control-case2" using 1:7 with lines  title "ay-with-const-with-pidcontrol-with-beta"















#Joint error data no constraint , PID control no stabilizer

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint pose (rad)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:2 with lines  title "j0-e-no-const-with-control",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:3 with lines  title "j1-e-no-const-with-control"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint rate (rad/s)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:4 with lines  title "v0-e-no-const-with-control",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:5 with lines  title "v1-e-no-const-with-control"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint acc (rad/s*s)"
plot "tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:6 with lines  title "a0-e-no-const-with-control",\
"tempJointDataCase14-Euler-Error-no-constraint-PID-control" using 1:7 with lines  title "a1-e-no-const-with-control"

#Joint actual data

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint pose (rad)"
plot "tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:2 with lines  title "j0-no-const-with-control",\
"tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:3 with lines  title "j1-no-const-with-control"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint rate (rad/s)"
plot "tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:4 with lines  title "v0-no-const-with-control",\
"tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:5 with lines  title "v1-no-const-with-control"


unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "joint acc (rad/s*s)"
plot "tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:6 with lines  title "a0-no-const-with-control",\
"tempJointDataCase14-Euler-Actual-no-constraint-PID-control" using 1:7 with lines  title "a1-no-const-with-control"



#Cartesian actual data

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian pose (m)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:2 with lines  title "x-no-const-with-control",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:3 with lines  title "y-no-const-with-control"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian vel (m/s)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:4 with lines  title "vx-no-const-with-control",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:5 with lines  title "vy-no-const-with-control"



unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:6 with lines  title "ax-no-const-with-control",\
"tempCartDataCase14-Euler-Actual-no-constraint-PID-control" using 1:7 with lines  title "ay-no-const-with-control"



# Cartesian error

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian pose (m)"
plot "tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:2 with lines  title "x-e-no-const-with-control",\
"tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:3 with lines  title "y-e-no-const-with-control"

unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian vel (m/s)"
plot "tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:4 with lines  title "vx-e-no-const-with-control",\
"tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:5 with lines  title "vy-e-no-const-with-control"



unset multiplot
set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian acc (m/s*s)"
plot "tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:6 with lines  title "ax-e-no-const-with-control",\
"tempCartDataCase14-Euler-Error-no-constraint-PID-control" using 1:7 with lines  title "ay-e-no-const-with-control"

