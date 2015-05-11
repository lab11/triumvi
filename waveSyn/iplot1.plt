
set term postscript enhanced eps color font "Helvetica,18" size 4in,4in

set output "IVSyn.eps"
set border 3 lw 2
set ytics nomirror
set xtics nomirror

set xlabel "time (ms)"
set ylabel "current (A)"
set grid

plot "timeVariant/data23.txt" every :::::120 u ($3/16000):($6/1000) title "measured current" w l ls 1 lc -1 axes x1y1 ,\
	"sineTable.txt" u ($1*1000):2 title 'synthesized voltage' w l ls 1 lc 4 axes x1y2


