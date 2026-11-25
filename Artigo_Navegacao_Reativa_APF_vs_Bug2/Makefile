TEX = pdflatex -interaction=nonstopmode -halt-on-error
BIB = bibtex

all: main.pdf

main.pdf: main.tex referencias.bib secoes/*.tex figuras/*.tex tabelas/*.tex
	$(TEX) main.tex
	$(BIB) main
	$(TEX) main.tex
	$(TEX) main.tex

clean:
	rm -f *.aux *.bbl *.blg *.fdb_latexmk *.fls *.log *.out *.synctex.gz *.toc

distclean: clean
	rm -f main.pdf

.PHONY: all clean distclean
