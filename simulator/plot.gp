plot 't.dat' with linespoints title "Water temperature °C", \
	't.dat' using 1:5 with linespoints title "PID output", \
	't.dat' using 1:($6 * 10) with lines title "Heating element"
pause -1
