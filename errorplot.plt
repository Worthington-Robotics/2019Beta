#####################################
# testPlot.plt
# Created By Cole Tucker 9/27/2018
#####################################

# Data Order:
#   1 Time
#   2 left position
#   3 left velocity
#   4 right position
#   5 right velocity
#   6 heading (deg)
#   7 error pose x
#   8 error pose y
#   9 error pose theta
#  10 master stick button 2
#  11 left demand velocity
#  12 right demand velocity
#  13 right distance traveled
#  14 left distance traveled
#  15 target pose x
#  16 target pose y
#  17 target pose theta
#  18 - 22 IDK


datafile = "data.txt"

set datafile separator ","
set grid
set title 'Robot Drive Data'
set xlabel "Time (s)"
set ytics nomirror
set autoscale  y

plot datafile using 1:($2*10) axes x1y1 with points title "left talon position (ticks)" lw 3 lc rgb "red", \
	 datafile using 1:($3*10) axes x1y1 with points title "left talon velocity (ticks / 100ms)" lw 3 lc rgb "green", \
	 datafile using 1:($4*10) axes x1y1 with points title "right talon position (ticks)" lw 3 lc rgb "blue", \
	 datafile using 1:($5*10) axes x1y1 with points title "right talon velocity (ticks / 100ms)" lw 3 lc rgb "brown", \
     datafile using 1:($6*01) axes x1y1 with points title "robot heading (degrees)" lw 3 lc rgb "purple", \
	 datafile using 1:($7*10) axes x1y1 with points title "error pose x (in)" lw 3 lc rgb "gold", \
     datafile using 1:($8*10) axes x1y1 with points title "error pose y (in)" lw 3 lc rgb "pink", \
     datafile using 1:($9*10) axes x1y1 with points title "error pose theta (deg)" lw 3 lc rgb "violet", \
     datafile using 1:($11*1) axes x1y1 with points title "left demand velocity (ticks / 100ms)" lw 3 lc rgb "cyan" , \
     datafile using 1:($12*1) axes x1y1 with points title "right demand velocity (ticks / 100ms)" lw 3 lc rgb "grey"