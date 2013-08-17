set yrange[-100:100]
set grid
set terminal wxt size 1600,1000
plot \
	't.dat' using 1:2 with lines title "Water (actual) temperature °C", \
	't.dat' using 1:3 with lines title "Sensor (sensed) temperature °C", \
	't.dat' using 1:($4*100) with lines title "Error temperature °C * 100", \
	't.dat' using 1:5 with lines title "PID output", \
	't.dat' using 1:($6 * 100) with lines title "Duty Cycle %", \
	't.dat' using 1:($7 * 10) with points title "Heating element", \
	't.dat' using 1:($9) with lines title "PID Integral"
pause -1
