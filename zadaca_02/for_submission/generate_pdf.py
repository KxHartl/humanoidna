import markdown2
from fpdf import FPDF, HTMLMixin
import sys
import codecs

class MyFPDF(FPDF, HTMLMixin):
    pass

def md_to_pdf(md_file, pdf_file):
    with codecs.open(md_file, 'r', encoding='utf-8') as f:
        md_text = f.read()

    # markdown2 to html
    html = markdown2.markdown(md_text)
    
    pdf = MyFPDF()
    pdf.add_page()
    # Add a unicode font if needed, otherwise default font might not support some Croatian characters (č,ć,ž,š,đ)
    # Using built-in font might fail. Let's try basic Helvetica but replace croatian chars if it fails, or just use core fonts
    try:
        # Default core fonts don't support utf8, we need to add a unicode font
        # If we don't have a ttf, we can just replace croatian letters to ascii for the pdf to avoid crash
        html = html.replace('č', 'c').replace('ć', 'c').replace('ž', 'z').replace('š', 's').replace('đ', 'dj')
        html = html.replace('Č', 'C').replace('Ć', 'C').replace('Ž', 'Z').replace('Š', 'S').replace('Đ', 'Dj')
        pdf.write_html(html)
        pdf.output(pdf_file)
        print("PDF created successfully.")
    except Exception as e:
        print(f"Error creating PDF: {e}")

if __name__ == "__main__":
    md_to_pdf("Kresimir_Hartl_Report.md", "Kresimir_Hartl_Report.pdf")
