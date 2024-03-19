import os
import numpy as np

indir = "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/3d_raw/os1/0"
outdir = "/home/warthog/LevaDetr/CUDA-CenterPoint/data/leva/test"

# Load the data
pc_files= os.listdir(indir)

for pc_file in pc_files:
    pc_path = os.path.join(indir, pc_file)

    # Load the point cloud
    pc_np = np.fromfile(pc_path, dtype=np.float32).reshape(-1, 5)

    # Convert time from ns to s
    pc_np[:, 4] /= 1e9

    # Reduce point cloud range
    min_bound = [-25, -25] # xmin,ymin
    max_bound = [25, 25] # xmax,ymax

    # Subsample point cloud to 32 channel resolution
    pc_np = pc_np.reshape(128, 1024, 5)
    pc_np = pc_np[::1, :, :].reshape(-1, 5)

    mask = (pc_np[:, 0] > min_bound[0]) & (pc_np[:, 0] < max_bound[0]) & (pc_np[:, 1] > min_bound[1]) & (pc_np[:, 1] < max_bound[1])
    pc_np = pc_np[mask]

    # Save the point cloud to outdir
    out_path = os.path.join(outdir, pc_file)
    pc_np.tofile(out_path)
    print(f"Saved {out_path}")