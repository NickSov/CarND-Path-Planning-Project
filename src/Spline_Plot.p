# Gnuplot for RMSE Error

set title "Spline Practice Plot" font "Helvetica,15"
set term x11

plot  "spline_curve.txt" using 1:2 title 'Lane Change No Smoothing' with linespoints,\
      "spline_curve.txt" using 1:3 title 'Lane Change Aggresive (w/ Spline)' with linespoints, \
      "spline_curve.txt" using 1:4 title 'Lane Change Gradual (w/ Spline)' with linespoints,
