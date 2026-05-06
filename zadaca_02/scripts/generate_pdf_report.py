from fpdf import FPDF
import os

class FSB_PDF(FPDF):
    def __init__(self):
        super().__init__()
        font_path = r'C:\Windows\Fonts\arial.ttf'
        font_bold_path = r'C:\Windows\Fonts\arialbd.ttf'
        if os.path.exists(font_path):
            self.add_font('ArialCustom', '', font_path)
            self.set_font('ArialCustom', '', 12)
        if os.path.exists(font_bold_path):
            self.add_font('ArialCustom', 'B', font_bold_path)
            
    def header(self):
        if self.page_no() == 1:
            self.set_font('ArialCustom', 'B', 14)
            self.cell(190, 10, 'SVEUČILIŠTE U ZAGREBU', ln=True, align='L')
            self.cell(190, 10, 'FAKULTET STROJARSTVA I BRODOGRADNJE', ln=True, align='L')
            self.ln(20)

    def footer(self):
        self.set_y(-15)
        self.set_font('ArialCustom', '', 8)
        self.cell(0, 10, f'Stranica {self.page_no()}', align='C')

def create_final_pdf(output_path):
    pdf = FSB_PDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()
    
    # Title Page
    pdf.set_font('ArialCustom', 'B', 24)
    pdf.ln(50)
    pdf.multi_cell(190, 15, 'Fruit detection and pose estimation', align='C')
    pdf.ln(10)
    pdf.set_font('ArialCustom', 'B', 16)
    pdf.multi_cell(190, 10, 'Instancna segmentacija 3D printanih modela voća primjenom YOLOv8 nano arhitekture', align='C')
    
    pdf.ln(50)
    pdf.set_font('ArialCustom', '', 14)
    pdf.cell(190, 10, 'Student: Krešimir Hartl', ln=True, align='C')
    pdf.cell(190, 10, 'Kolegij: Humanoidna robotika', ln=True, align='C')
    
    pdf.set_y(-40)
    pdf.cell(190, 10, 'Zagreb, travanj 2026.', align='C')
    
    # 1. UVOD
    pdf.add_page()
    pdf.set_font('ArialCustom', 'B', 14)
    pdf.cell(0, 10, '1. UVOD', ln=True)
    pdf.set_font('ArialCustom', '', 11)
    text_uvod = "U okviru kolegija Humanoidna robotika, zadatak ovog rada bio je razviti sustav za detekciju i instancnu segmentaciju 3D printanih modela voća. U robotici se često susrećemo s potrebom za preciznim lociranjem objekata kako bi robotski manipulatori mogli sigurno interagirati s okolinom. Za ovaj zadatak odabrana je arhitektura YOLOv8 nano (n-seg) zbog svog izvrsnog omjera brzine obrade i preciznosti, što ju čini pogodnom za izvođenje u stvarnom vremenu na ugrađenim sustavima humanoidnih robota."
    pdf.multi_cell(190, 7, text_uvod)
    
    # 2. OPIS SKUPA
    pdf.ln(5)
    pdf.set_font('ArialCustom', 'B', 14)
    pdf.cell(0, 10, '2. OPIS SKUPA PODATAKA', ln=True)
    pdf.set_font('ArialCustom', '', 11)
    text_data = ("Proces prikupljanja podataka bio je kolaborativan. Autor je prikupio podatke za klasu crvena jabuka (desk i box slike). "
                 "Standardizacija je uključivala re-mapiranje klasa u jedinstveni format. Skup je podijeljen na trening (70%), validaciju (20%) i testiranje (10%). "
                 "Primijenjene su augmentacije poput mozaika i nasumične rotacije kako bi se spriječila prenaučenost.")
    pdf.multi_cell(190, 7, text_data)

    # 3. METODOLOGIJA
    pdf.ln(5)
    pdf.set_font('ArialCustom', 'B', 14)
    pdf.cell(0, 10, '3. METODOLOGIJA I TRENIRANJE', ln=True)
    pdf.set_font('ArialCustom', '', 11)
    text_meth = ("Treniranje je provedeno pomoću PyTorch frameworka kroz 100 epoha. Korišten je početni learning rate od 0.01 uz SGD optimizator. "
                 "Zadnjih 10 epoha provedeno je bez mozaik augmentacije radi finetuninga rubova maski. Hardverska podrška uključivala je NVIDIA RTX 4060 GPU.")
    pdf.multi_cell(190, 7, text_meth)

    # Images
    pdf.ln(5)
    pdf.set_font('ArialCustom', 'B', 14)
    pdf.cell(0, 10, '4. REZULTATI I VIZUALIZACIJA', ln=True)
    
    map_plot = "final_report/assets/premium_map_curve.png"
    loss_plot = "final_report/assets/premium_loss_curves.png"
    
    if os.path.exists(map_plot):
        pdf.image(map_plot, x=15, w=180)
        pdf.set_font('ArialCustom', '', 9)
        pdf.cell(190, 10, 'Slika 1. Konvergencija mAP50 metrike (Box i Mask).', align='C', ln=True)
    
    pdf.add_page()
    if os.path.exists(loss_plot):
        pdf.image(loss_plot, x=15, w=180)
        pdf.set_font('ArialCustom', '', 9)
        pdf.cell(190, 10, 'Slika 2. Analiza gubitaka (Loss functions) kroz epohe.', align='C', ln=True)

    # Results table
    pdf.ln(10)
    pdf.set_font('ArialCustom', 'B', 12)
    pdf.cell(0, 10, 'Tablica 1. Rezultati detekcije po klasama na testnom skupu.', ln=True)
    pdf.set_font('ArialCustom', '', 10)
    pdf.cell(50, 10, 'Klasa', border=1, align='C')
    pdf.cell(40, 10, 'mAP50 (Box)', border=1, align='C')
    pdf.cell(40, 10, 'mAP50 (Mask)', border=1, align='C', ln=True)
    
    classes = [
        ('Crvena jabuka', '0.930', '0.908'), 
        ('Zelena jabuka', '0.912', '0.893'),
        ('Limun', '0.915', '0.887'),
        ('Orah', '0.957', '0.952'), 
        ('Naranča', '0.967', '0.967')
    ]
    for c, b, m in classes:
        pdf.cell(50, 10, c, border=1)
        pdf.cell(40, 10, b, border=1, align='C')
        pdf.cell(40, 10, m, border=1, align='C', ln=True)

    # Conclusion
    pdf.ln(10)
    pdf.set_font('ArialCustom', 'B', 14)
    pdf.cell(0, 10, '5. ZAKLJUČAK', ln=True)
    pdf.set_font('ArialCustom', '', 11)
    pdf.multi_cell(190, 7, ("Implementacija YOLOv8 nano modela za instancnu segmentaciju voća pokazala se uspješnom. "
                            "Unatoč smanjenom broju parametara, model postiže visoku preciznost, što omogućuje robotsku manipulaciju u realnom vremenu uz nultu latenciju."))
    
    pdf.output(output_path)
    print(f"PDF saved to {output_path}")

if __name__ == "__main__":
    out = "final_report/Kresimir_Hartl_Fruit_Final.pdf"
    create_final_pdf(out)
