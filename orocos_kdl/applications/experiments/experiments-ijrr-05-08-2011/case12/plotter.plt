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
set ylabel "joint pose (rad)"
plot "tempJointDataCase12-Euler-conf" using 1:2 with lines  title "joint0-euler", "tempJointDataCase12-Euler-conf" using 1:3 with lines title "joint1-euler", "tempJointDataCase12-AB2-AM3-conf" using 1:2 with lines  title "joint0-ab2am3", "tempJointDataCase12-AB2-AM3-conf" using 1:3 with lines title "joint1-ab2am3"

unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint rate (rad/s)"
plot "tempJointDataCase12-Euler-conf" using 1:4 with lines  title "joint0-euler", "tempJointDataCase12-Euler-conf" using 1:5 with lines title "joint1-euler", "tempJointDataCase12-AB2-AM3-conf" using 1:4 with lines  title "joint0-ab2am3", "tempJointDataCase12-AB2-AM3-conf" using 1:5 with lines title "joint1-ab2am3"



unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "joint acc (rad/sec^2)"
plot "tempJointDataCase12-Euler-conf" using 1:6 with lines  title "joint0-euler", "tempJointDataCase12-Euler-conf" using 1:7 with lines title "joint1-euler", "tempJointDataCase12-AB2-AM3-conf" using 1:6 with lines  title "joint0-ab2am3", "tempJointDataCase12-AB2-AM3-conf" using 1:7 with lines title "joint1-ab2am3"


unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "EE position (m)"
plot "tempJointDataCase12-Euler-conf" using 1:10 with lines  title "x pose-euler", "tempJointDataCase12-Euler-conf" using 1:11 with lines title "y pose-euler", "tempJointDataCase12-AB2-AM3-conf" using 1:10 with lines  title "x pose-ab2am3", "tempJointDataCase12-AB2-AM3-conf" using 1:11 with lines title "y pose-ab2am3"



unset multiplot
set size 1,1
set origin 0,0
set multiplot


set origin 0.0,0.0
set xlabel "time (sec)"
set ylabel "joint torque (Nm)"
plot "tempJointDataCase12-Euler-conf" using 1:8 with lines  title "joint0-euler", "tempJointDataCase12-Euler-conf" using 1:9 with lines title "joint1-euler", "tempJointDataCase12-AB2-AM3-conf" using 1:8 with lines  title "joint0-ab2am3", "tempJointDataCase12-AB2-AM3-conf" using 1:9 with lines title "joint1-ab2am3"





unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian accX (m/s^2)"
plot "tempCartDataCase12-Euler-conf" using 1:5 with lines  title "link1-euler", "tempCartDataCase12-AB2-AM3-conf" using 1:5 with lines title "link1-ab2am3"





unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian accY (m/s^2)"
plot "tempCartDataCase12-Euler-conf" using 1:6 with lines  title "link1-euler", "tempCartDataCase12-AB2-AM3-conf" using 1:6 with lines title "link1-ab2am3"




unset multiplot
set size 1,1
set origin 0,0
set multiplot

set origin 0,0
set xlabel "time (sec)"
set ylabel "cartesian accZ (m/s^2)"
plot "tempCartDataCase12-Euler-conf" using 1:7 with lines  title "link1-euler", "tempCartDataCase12-AB2-AM3-conf" using 1:7 with lines title "link1-ab2am3"

