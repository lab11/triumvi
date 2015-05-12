
set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in

set output "IVSyn.eps"
set border 3 lw 2
set ytics nomirror
set xtics nomirror

set xlabel "time (ms)"
set ylabel "current (A)"
set grid

set xrange [0:17]

set multiplot
set origin 0,0

currentOffset = 0.7

plot "timeVariant/data27.txt" every :::::120 u ($3/16000):($6/1000-currentOffset) title "measured current" w l ls 1 lc -1 axes x1y1 ,\
	"sineTable.txt" u ($1*1000):2 title 'synthesized voltage' w l ls 1 lc 4 axes x1y2

set origin 0,0
set datafile separator ","
plot "T0115CH4.CSV" every::17 u ($1*1000):($2*-6.39375) title "WaveC" w l ls 1 lc 7 axes x1y1
unset multiplot


