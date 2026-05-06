import os

# Define the new mapping
# Old indices: 0: Banana, 1: Crvena jabuka, 2: Limun, 3: Naranca, 4: Orah, 5: Zelena jabuka
# New indices (trying to follow requested order):
# 0: red apple (Old 1)
# 1: green apple (Old 5)
# 2: lemon (Old 2)
# 3: banana (Old 0)
# 4: avocado (Empty)
# 5: walnut (Old 4)
# 6: pear (Empty)
# 7: orange (Old 3)

remap = {
    0: 3, # Banana
    1: 0, # Crvena jabuka
    2: 2, # Limun
    3: 7, # Naranca
    4: 5, # Orah
    5: 1  # Zelena jabuka
}

data_root = "datasets/Humanoidna_svo_voce_02.yolov8-obb"

for split in ['train', 'valid', 'test']:
    label_dir = os.path.join(data_root, split, 'labels')
    if not os.path.exists(label_dir):
        continue
    
    print(f"Remapping labels in {split}...")
    for file in os.listdir(label_dir):
        if file.endswith(".txt"):
            path = os.path.join(label_dir, file)
            with open(path, "r") as f:
                lines = f.readlines()
            
            new_lines = []
            for line in lines:
                parts = line.split()
                if parts:
                    old_idx = int(parts[0])
                    new_idx = remap.get(old_idx, old_idx)
                    parts[0] = str(new_idx)
                    new_lines.append(" ".join(parts) + "\n")
            
            with open(path, "w") as f:
                f.writelines(new_lines)

print("Label remapping complete.")
