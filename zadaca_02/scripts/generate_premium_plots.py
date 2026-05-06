import pandas as pd
import matplotlib.pyplot as plt
import os

# Set professional plotting style
plt.style.use('seaborn-v0_8-muted')
plt.rcParams['font.family'] = 'serif'
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.3
plt.rcParams['axes.spines.top'] = False
plt.rcParams['axes.spines.right'] = False

def create_premium_plots(csv_path, output_dir):
    df = pd.read_csv(csv_path)
    # Clean column names (strip whitespace)
    df.columns = [c.strip() for c in df.columns]
    
    epochs = df['epoch']
    
    # 1. mAP Comparison Plot
    plt.figure(figsize=(10, 6))
    plt.plot(epochs, df['metrics/mAP50(B)'], label='mAP50 (Box)', color='#2E5A88', linewidth=2)
    plt.plot(epochs, df['metrics/mAP50(M)'], label='mAP50 (Mask)', color='#A63D40', linewidth=2, linestyle='--')
    plt.xlabel('Epoch', fontsize=12)
    plt.ylabel('mAP', fontsize=12)
    plt.title('Preciznost modela kroz epohe (mAP50)', fontsize=14, fontweight='bold', pad=20)
    plt.legend(frameon=True, fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'premium_map_curve.png'), dpi=300)
    plt.close()

    # 2. Loss Curves (Cls & Box)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Box Loss
    ax1.plot(epochs, df['train/box_loss'], label='Train Box Loss', color='#4E937A')
    ax1.plot(epochs, df['val/box_loss'], label='Val Box Loss', color='#F2BB05')
    ax1.set_title('Gubitak lokacije (Box Loss)', fontsize=13)
    ax1.set_xlabel('Epoch')
    ax1.legend()
    
    # Cls Loss
    ax2.plot(epochs, df['train/cls_loss'], label='Train Cls Loss', color='#4E937A')
    ax2.plot(epochs, df['val/cls_loss'], label='Val Cls Loss', color='#F2BB05')
    ax2.set_title('Gubitak klasifikacije (Cls Loss)', fontsize=13)
    ax2.set_xlabel('Epoch')
    ax2.legend()
    
    plt.suptitle('Analiza konvergencije gubitaka', fontsize=15, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(os.path.join(output_dir, 'premium_loss_curves.png'), dpi=300)
    plt.close()
    
    print(f"Premium plots saved to {output_dir}")

if __name__ == "__main__":
    csv_path = "output/runs/fruit_seg_experiment/results.csv"
    output_dir = "final_report/assets"
    os.makedirs(output_dir, exist_ok=True)
    
    if os.path.exists(csv_path):
        create_premium_plots(csv_path, output_dir)
    else:
        print(f"Error: csv not found at {csv_path}")
