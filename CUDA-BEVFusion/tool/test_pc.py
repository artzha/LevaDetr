import os
from os.path import join
import numpy as np
# from PIL import Image

# import torch
import tensor

def numpy_to_pcd(array, filename):
    """
    Convert a Nx3 numpy array to a PCD file.

    Parameters:
    array (numpy.ndarray): Nx3 numpy array containing point cloud data.
    filename (str): The output filename for the PCD file.
    """
    import numpy as np
    assert array.shape[1] == 3, "Input array must be Nx3."

    header = f"""# .PCD v0.7 - Point Cloud Data file format
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH {len(array)}
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS {len(array)}
    DATA ascii
    """
    with open(filename, 'w') as f:
        f.write(header)
        np.savetxt(f, array, fmt='%f %f %f')

seq = 0
frame=0
root_dir = "/robodata/ecocar_logs/processed/CACCDataset"
pc_dir = f'{root_dir}/3d_raw/os1/{seq}'
pc_file = f'3d_raw_os1_{seq}_{frame}.bin'
pc_path = join(pc_dir, pc_file)

cwd = os.getcwd()
pc_outfile = pc_file.replace("bin", "tensor")
pc_outpath = f'{cwd}/{pc_outfile}'

pc_np = np.fromfile(pc_path, dtype=np.float32).reshape(-1, 5).astype(np.float16)
x = pc_np[:, 0]
y = pc_np[:, 1]

pc_np[:, 0] = y
pc_np[:, 1] = x
pc_np[:, 2] = -pc_np[:, 2]

# pc_th = torch.tensor(pc_np).flatten().half()

print(f'Saved to {pc_outpath}')
tensor.save(pc_np, pc_outpath)

numpy_to_pcd(pc_np[:, :3], f'{cwd}/{pc_file.replace("bin", "pcd")}')
