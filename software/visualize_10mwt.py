import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ast

# ==========================================
# CONFIGURATION
# ==========================================
# Point this to any trial folder in the /data directory
TRIAL_DIR = "../data/P03_slow_1" 

print(f"Loading synchronized datasets for {os.path.basename(TRIAL_DIR)}...")

# 1. Load the files
df_insole = pd.read_csv(f"{TRIAL_DIR}/FINAL_insole_10mwt.csv")
df_phy = pd.read_csv(f"{TRIAL_DIR}/FINAL_phyphox_10mwt.csv")
df_rl = pd.read_csv(f"{TRIAL_DIR}/FINAL_rawlogger_10mwt.csv")

# 2. Reset time to 0.0 seconds
df_insole['t_zeroed'] = df_insole['unix_time'] - df_insole['unix_time'].iloc[0]
df_phy['t_zeroed'] = df_phy['Time (s)'] - df_phy['Time (s)'].iloc[0]
df_rl['t_zeroed'] = df_rl['t_relative'] - df_rl['t_relative'].iloc[0]

# 3. Calculate Accelerometer Magnitudes
df_insole['accel_mag'] = np.sqrt(df_insole['lax']**2 + df_insole['lay']**2 + df_insole['laz']**2)
df_phy['accel_mag'] = np.sqrt(df_phy['Acceleration x (m/s^2)']**2 + df_phy['Acceleration y (m/s^2)']**2 + df_phy['Acceleration z (m/s^2)']**2)
df_rl['accel_mag'] = np.sqrt(df_rl['SensorAccelerationX_HD']**2 + df_rl['SensorAccelerationY_HD']**2 + df_rl['SensorAccelerationZ_HD']**2)

# 4. Calculate Total Foot Pressure
pressure_cols = ['p0', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9']
df_insole['total_pressure'] = df_insole[pressure_cols].sum(axis=1)

# Find shortest end time across datasets
max_time = min(
    df_insole['t_zeroed'].max(),
    df_phy['t_zeroed'].max(),
    df_rl['t_zeroed'].max()
)

# ==========================================
# 5. PARSE GROUND TRUTH ANNOTATIONS
# ==========================================
left_times, right_times = [], []
gt_path = f"{TRIAL_DIR}/ground_truth.txt"
if os.path.exists(gt_path):
    with open(gt_path, "r") as f:
        for line in f:
            if line.startswith("left_times"):
                left_times = ast.literal_eval(line.split("=")[1].strip())
            elif line.startswith("right_times"):
                right_times = ast.literal_eval(line.split("=")[1].strip())
else:
    print(f"[WARNING] No ground_truth.txt found in {TRIAL_DIR}")

# 6. Build the Master Plot
fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
(ax1, ax2, ax3, ax4) = axes

# Add the main title based on the folder name
trial_name = os.path.basename(TRIAL_DIR).replace("_", " ").title()
fig.suptitle(f"Ground Truth Synchronization: {trial_name}", fontsize=16, fontweight='bold')

# Panel 1: Phyphox (~400 Hz)
ax1.plot(df_phy['t_zeroed'], df_phy['accel_mag'], color='red', linewidth=1)
ax1.set_title('Smartphone IMU (~400 Hz) - Center of Mass Acceleration')
ax1.set_ylabel('Accel (m/s²)')
ax1.grid(True)

# Panel 2: Insole IMU (100 Hz)
ax2.plot(df_insole['t_zeroed'], df_insole['accel_mag'], color='purple', linewidth=1.5)
ax2.set_title('Smart Insole IMU (100 Hz) - Left Foot Acceleration')
ax2.set_ylabel('Accel (g)')
ax2.grid(True)

# Panel 3: Insole Pressure (100 Hz)
ax3.plot(df_insole['t_zeroed'], df_insole['total_pressure'], color='blue', linewidth=1.5)
ax3.fill_between(df_insole['t_zeroed'], df_insole['total_pressure'], color='blue', alpha=0.2)
ax3.set_title('Smart Insole Pressure (100 Hz) - Left Foot Weight Bearing Force')
ax3.set_ylabel('Total Pressure')
ax3.grid(True)

# Panel 4: Garmin RawLogger (25 Hz)
ax4.plot(df_rl['t_zeroed'], df_rl['accel_mag'], color='green', linewidth=1.5, marker='o', markersize=3)
ax4.set_title('Garmin RawLogger (25 Hz) - Right Wrist Acceleration')
ax4.set_xlabel('Steady-State Walking Time (Seconds)')
ax4.set_ylabel('Accel (mG)')
ax4.grid(True)

# Set tight shared x-axis bounds
ax4.set_xlim(0, max_time)

# Overlay Ground Truth Lines & remove default outer margins
for ax in axes:
    ax.margins(x=0)
    for lt in left_times:
        ax.axvline(x=lt, color='blue', linestyle='-', linewidth=2, alpha=0.5, label='Left Foot (Insole)' if lt == left_times[0] else "")
    for rt in right_times:
        ax.axvline(x=rt, color='darkorange', linestyle='--', linewidth=2, alpha=0.7, label='Right Foot' if rt == right_times[0] else "")
        
# Add legend only to the top plot
ax1.legend(loc='upper right')

# Compact layout for document insertion
plt.tight_layout(rect=[0, 0.02, 1, 0.97])
plt.show()