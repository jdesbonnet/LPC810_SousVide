set yrange[-40:100]
set grid
plot \
	't.dat' using 1:2 with lines title "Water (actual) temperature °C", \
	't.dat' using 1:3 with lines title "Sensor (sensed) temperature °C", \
	't.dat' using 1:5 with lines title "PID output", \
	't.dat' using 1:($6 * 100) with lines title "Duty Cycle %", \
	't.dat' using 1:($7 * 10) with lines title "Heating element"
pause -1
