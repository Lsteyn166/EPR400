##
##  Department of Electrical, Electronic and Computer Engineering.
##  EPR400/2 Project Proposal - Makefile.
##  Copyright (C) 2011-2017 University of Pretoria.
##

all: proposal.pdf

proposal.pdf: *.tex *.bib epr400.cls
	pdflatex proposal
	bibtex   proposal
	pdflatex proposal
	pdflatex proposal

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc
	rm -f *converted-to.pdf proposal.pdf

## End of File.
