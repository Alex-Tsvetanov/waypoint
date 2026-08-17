# pdfLaTeX, НЕ XeLaTeX (T2A + tempora). Форматирането е в preamble.tex.
$pdf_mode = 1;
$bibtex_use = 2;
$out_dir = 'build';
$clean_ext = 'bbl nav out snm synctex.gz run.xml';

# latexmk изпълнява bibtex ВЪТРЕ в $out_dir. Оттам references.bib не се вижда и
# MiKTeX bibtex произвежда празен .bbl БЕЗ съобщение за грешка: в дневника пише
# „Database file #1: references.bib“, но „You've used 0 entries“, а списъкът с
# литература излиза празен. Проверено на тази машина. Пътят се добавя изрично.
ensure_path('BIBINPUTS', '.', '..');
