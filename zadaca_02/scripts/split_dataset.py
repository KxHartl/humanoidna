import os
import shutil
import random

def split_dataset(data_root, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1):
    # Ensure ratios sum to 1
    total_total = train_ratio + val_ratio + test_ratio
    train_ratio /= total_total
    val_ratio /= total_total
    test_ratio /= total_total

    # 1. Collect ALL images and labels from existing splits to redistribute
    temp_dir = os.path.join(data_root, 'temp_unified_final')
    os.makedirs(os.path.join(temp_dir, 'images'), exist_ok=True)
    os.makedirs(os.path.join(temp_dir, 'labels'), exist_ok=True)

    all_found_images = []
    
    # We look in train, valid, and test
    for split in ['train', 'valid', 'test']:
        img_dir = os.path.join(data_root, split, 'images')
        lbl_dir = os.path.join(data_root, split, 'labels')
        
        if os.path.exists(img_dir):
            images = [f for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.png', '.jpeg'))]
            for img in images:
                src_img = os.path.join(img_dir, img)
                dst_img = os.path.join(temp_dir, 'images', img)
                
                # Check for label
                label_name = os.path.splitext(img)[0] + '.txt'
                src_lbl = os.path.join(lbl_dir, label_name)
                
                if os.path.exists(src_lbl):
                    shutil.move(src_img, dst_img)
                    shutil.move(src_lbl, os.path.join(temp_dir, 'labels', label_name))
                    all_found_images.append(img)
                else:
                    print(f"Skipping {img} (no label found)")

    # 2. Shuffle
    random.shuffle(all_found_images)
    num_total = len(all_found_images)
    num_train = int(num_total * train_ratio)
    num_val = int(num_total * val_ratio)

    splits = {
        'train': all_found_images[:num_train],
        'valid': all_found_images[num_train:num_train+num_val],
        'test': all_found_images[num_train+num_val:]
    }

    # 3. Create target directories (clearing them first if they exist to be safe)
    # Actually, we already moved everything out, so they should be empty except for maybe empty folders
    for split in ['train', 'valid', 'test']:
        os.makedirs(os.path.join(data_root, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(data_root, split, 'labels'), exist_ok=True)

    # 4. Distribute
    print(f"Redistributing {num_total} images with ratio {train_ratio:.1f}/{val_ratio:.1f}/{test_ratio:.1f}...")
    for split, images in splits.items():
        print(f"  - {split}: {len(images)} images")
        for img in images:
            label = os.path.splitext(img)[0] + '.txt'
            shutil.move(os.path.join(temp_dir, 'images', img), os.path.join(data_root, split, 'images', img))
            shutil.move(os.path.join(temp_dir, 'labels', label), os.path.join(data_root, split, 'labels', label))

    # Cleanup temp
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    print("Dataset re-splitting complete.")

if __name__ == "__main__":
    data_root = "datasets/Humanoidna_svo_voce_02.yolov8-obb"
    split_dataset(data_root, 0.7, 0.2, 0.1)
