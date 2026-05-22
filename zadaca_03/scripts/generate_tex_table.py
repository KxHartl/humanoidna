import json
import os

def generate_tex_table():
    json_path = 'data/processed/last_trajectory.json'
    out_path = 'dist/assets/trajectory_table.tex'
    
    if not os.path.exists(json_path):
        print("Trajectory file not found!")
        return
        
    with open(json_path, 'r') as f:
        data = json.load(f)
        
    points = data['points']
    
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("\\begin{longtable}{|c|c|c|c|c|c|c|}\n")
        f.write("\\caption{Sve generirane točke kompletne izvedbene trajektorije s korakom diskretizacije $dt = 0.05$ s.} \\label{tab:all_points} \\\\\n")
        f.write("\\hline\n")
        f.write("\\textbf{Vrijeme $t$ [s]} & \\textbf{$X$ [m]} & \\textbf{$Y$ [m]} & \\textbf{$Z$ [m]} & \\textbf{$R_X$ [rad]} & \\textbf{$R_Y$ [rad]} & \\textbf{$R_Z$ [rad]} \\\\\n")
        f.write("\\hline\n")
        f.write("\\endfirsthead\n\n")
        
        f.write("\\multicolumn{7}{c}%\n")
        f.write("{{\\bfseries \\tablename\\ \\thetable{} -- nastavak s prethodne stranice}} \\\\\n")
        f.write("\\hline\n")
        f.write("\\textbf{Vrijeme $t$ [s]} & \\textbf{$X$ [m]} & \\textbf{$Y$ [m]} & \\textbf{$Z$ [m]} & \\textbf{$R_X$ [rad]} & \\textbf{$R_Y$ [rad]} & \\textbf{$R_Z$ [rad]} \\\\\n")
        f.write("\\hline\n")
        f.write("\\endhead\n\n")
        
        f.write("\\hline \\multicolumn{7}{|r|}{{Nastavak na sljedećoj stranici...}} \\\\\n")
        f.write("\\hline\n")
        f.write("\\endfoot\n\n")
        
        f.write("\\hline\n")
        f.write("\\endlastfoot\n\n")
        
        for p in points:
            t = p['t']
            pos = p['pos']
            f.write(f"{t:.2f} & {pos[0]:.4f} & {pos[1]:.4f} & {pos[2]:.4f} & {pos[3]:.4f} & {pos[4]:.4f} & {pos[5]:.4f} \\\\\n")
            
        f.write("\\end{longtable}\n")
        
if __name__ == '__main__':
    generate_tex_table()
