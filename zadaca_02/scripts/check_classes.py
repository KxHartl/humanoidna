import os
from collections import Counter

label_dir = "datasets/Humanoidna_svo_voce_02.yolov8-obb/train/labels"
classes = []

for file in os.listdir(label_dir):
    if file.endswith(".txt"):
        with open(os.path.join(label_dir, file), "r") as f:
            for line in f:
                parts = line.split()
                if parts:
                    classes.append(int(parts[0]))

print(Counter(classes))
