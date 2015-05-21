
set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in

set border 11 lw 2
set xtics nomirror
set ytics nomirror

set xlabel "Ground Truth Power (W)"
set ylabel "Measured Power (W)"
set y2label "error (%)"
set y2tics
set grid

set key out center bottom 


set output "BK0.eps"
plot "BK0proc.txt" u 1:2 title "PF = 1, CF = 1.414" w linespoints ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints ls 7 lc 4 axes x1y2

set output "BK2.eps"
plot "BK2proc.txt" u 1:2 title "PF = 1, CF = 3" w linespoints ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints ls 7 lc 4 axes x1y2

set output "BK3.eps"
plot "BK3proc.txt" u 1:2 title "PF = -0.85, CF = 2" w linespoints  ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints  ls 7 lc 4 axes x1y2

set output "BK5.eps"
plot "BK5proc.txt" u 1:2 title "PF = -0.5, CF = 3.5" w linespoints  ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints  ls 7 lc 4 axes x1y2

set output "BK6.eps"
plot "BK6proc.txt" u 1:2 title "PF = 0.85, CF = 2" w linespoints  ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints  ls 7 lc 4 axes x1y2

set output "BK8.eps"
plot "BK8proc.txt" u 1:2 title "PF = 0.5, CF = 3.5" w linespoints  ls 4 lc -1 axes x1y1 ,\
	"" u 1:(($2-$1)/$1*100) title 'Error' w linespoints  ls 7 lc 4 axes x1y2
