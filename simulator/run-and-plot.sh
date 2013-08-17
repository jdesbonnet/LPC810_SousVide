#!/bin/bash
#
# Run simulation and plot result with GNUPlot
# Add PID parameters Kp Ki Kd as arguments
# eg bash run-and-plot.sh 2 0.0001 500
#
./water-pid-simulator $@ > t.dat
gnuplot plot.gp
