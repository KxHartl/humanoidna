import json
import os
import shutil
import yaml
from pathlib import Path

def prepare_dataset():
    # Paths
    base_data_path = Path("data/datasets")
    coco_dataset_path = base_data_path / "voce_gotovo.v2i.coco-segmentation"
    yolo_dataset_path = base_data_path / "yolo_seg_dataset"
    classes_file = base_data_path / "classes.txt"
    
    # 1. Read desired classes
    with open(classes_file, "r", encoding="utf-8") as f:
        desired_classes = [line.strip() for line in f if line.strip()]
    
    class_name_to_new_id = {name: i for i, name in enumerate(desired_classes)}
    print(f"Target classes mapping: {class_name_to_new_id}")

    # Create YOLO structure
    for split in ["train", "valid", "test"]:
        (yolo_dataset_path / "images" / split).mkdir(parents=True, exist_ok=True)
        (yolo_dataset_path / "labels" / split).mkdir(parents=True, exist_ok=True)

    # 2. Process each split
    for split in ["train", "valid", "test"]:
        print(f"Processing split: {split}")
        split_path = coco_dataset_path / split
        anno_file = split_path / "_annotations.coco.json"
        
        if not anno_file.exists():
            print(f"Warning: Annotation file {anno_file} not found. Skipping split.")
            continue
            
        with open(anno_file, "r", encoding="utf-8") as f:
            coco_data = json.load(f)
            
        # Map old category IDs to new IDs
        old_id_to_new_id = {}
        for cat in coco_data["categories"]:
            old_name = cat["name"]
            # Flexible matching (case-insensitive, strip whitespace)
            matched_new_name = next((n for n in desired_classes if n.lower() == old_name.lower()), None)
            if matched_new_name:
                old_id_to_new_id[cat["id"]] = class_name_to_new_id[matched_new_name]
            else:
                print(f"Warning: Category '{old_name}' in COCO JSON not found in classes.txt. It will be ignored.")

        # Organize images by ID
        images = {img["id"]: img for img in coco_data["images"]}
        
        # Collect annotations per image
        img_annotations = {}
        for anno in coco_data["annotations"]:
            img_id = anno["image_id"]
            cat_id = anno["category_id"]
            
            if cat_id in old_id_to_new_id:
                if img_id not in img_annotations:
                    img_annotations[img_id] = []
                
                new_cat_id = old_id_to_new_id[cat_id]
                segmentation = anno["segmentation"]
                
                # Check if it's a polygon (list of lists or list)
                if isinstance(segmentation, list):
                    for poly in segmentation:
                        if len(poly) >= 6: # At least 3 points
                            img_annotations[img_id].append((new_cat_id, poly))
        
        # 3. Save images and labels
        for img_id, img_info in images.items():
            img_filename = img_info["file_name"]
            src_img_path = split_path / img_filename
            
            if not src_img_path.exists():
                # Try finding it with same name but in case Roboflow messed up extension
                print(f"Warning: Image {src_img_path} not found.")
                continue

            # Copy image
            dest_img_path = yolo_dataset_path / "images" / split / img_filename
            shutil.copy(src_img_path, dest_img_path)
            
            # Write label txt
            label_filename = Path(img_filename).stem + ".txt"
            dest_label_path = yolo_dataset_path / "labels" / split / label_filename
            
            w, h = img_info["width"], img_info["height"]
            
            with open(dest_label_path, "w", encoding="utf-8") as f_label:
                if img_id in img_annotations:
                    for new_cat_id, poly in img_annotations[img_id]:
                        # Normalize coordinates
                        norm_poly = []
                        for i in range(0, len(poly), 2):
                            norm_poly.append(str(poly[i] / w))
                            norm_poly.append(str(poly[i+1] / h))
                        
                        f_label.write(f"{new_cat_id} {' '.join(norm_poly)}\n")

    # 4. Create data_seg.yaml
    yaml_content = {
        "train": str((yolo_dataset_path / "images" / "train").absolute()),
        "val": str((yolo_dataset_path / "images" / "valid").absolute()),
        "test": str((yolo_dataset_path / "images" / "test").absolute()),
        "nc": len(desired_classes),
        "names": desired_classes
    }
    
    with open(yolo_dataset_path / "data_seg.yaml", "w", encoding="utf-8") as f_yaml:
        yaml.dump(yaml_content, f_yaml, default_flow_style=False, sort_keys=False)
        
    print(f"Dataset preparation complete. Saved to {yolo_dataset_path}")
    print(f"YAML config created at {yolo_dataset_path / 'data_seg.yaml'}")

if __name__ == "__main__":
    prepare_dataset()
