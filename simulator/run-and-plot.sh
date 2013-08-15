#!/bin/bash
#
# Run simulation and plot result with GNUPlot
# Add PID parameters Kp Ki Kd as arguments
#
./water-pid-simulator $@ > t.dat
gnuplot plot.gp
