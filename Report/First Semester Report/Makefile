##
##  Department of Electrical, Electronic and Computer Engineering.
##  EPR400/2 Project Proposal - Makefile.
##  Copyright (C) 2011-2017 University of Pretoria.
##

all: FirstSem.pdf

proposal.pdf: *.tex *.bib epr400.cls
	pdflatex FirstSem
	bibtex   FirstSem
	pdflatex FirstSem
	pdflatex FirstSem

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc
	rm -f *converted-to.pdf FirstSem.pdf

## End of File.
