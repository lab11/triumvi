
all:
	python dataProc.py
	gnuplot dataPlot.plt
	eps2png -f BK0.eps
	eps2png -f BK2.eps
	eps2png -f BK3.eps
	eps2png -f BK5.eps
	eps2png -f BK6.eps
	eps2png -f BK8.eps
	rm -f *.eps

clean:
	rm -f *.eps
	rm -f *.png
	rm -f *.txt
