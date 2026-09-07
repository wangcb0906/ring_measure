import os.path
import matplotlib.pyplot as plt
import numpy as np
import sys

def load_scan_binary(filename):
    """Reads the custom C++ binary file containing dimensions,
    metadata, and the 3D multi_array payload.
    Shape of the multi_array is (NUM, size, size, metrics_len).
    Binary file structure:
    - 4 size_t values for dimensions (dim0, dim1, dim2, metrics_len)(8)
    - 4 float values for metadata (da, di, h, table_h)(4)
    - Payload data as a flat array of floats(4)
    """
    with open(filename, "rb") as f:
        # 1. Read the 4 dimensions (size_t on a 64-bit system is uint64)
        shape = np.fromfile(f, dtype=np.uint64, count=4)

        # 2. Read the 4 metadata floats (da, di, h, table_h)
        metadata_arr = np.fromfile(f, dtype=np.float32, count=4)
        # 3. Read the payload data
        payload = np.fromfile(f, dtype=np.float32, count=np.prod(shape)).reshape(shape)

    return [metadata_arr, payload]

def load_recon_binary(filename):
    """Reads the custom C++ binary file containing dimensions,
    metadata, and the 3D multi_array payload.
    Shape of the multi_array is (size, size, metrics_len).
    Binary file structure:
    - 3 size_t values for dimensions (dim0, dim1, metrics_len)(8)
    - 4 float values for metadata (da, di, h, table_h)(4)
    - Payload data as a flat array of floats(4)
    """
    with open(filename, "rb") as f:
        # 1. Read the 3 dimensions (size_t on a 64-bit system is uint64)
        shape = np.fromfile(f, dtype=np.uint64, count=3)
  
        # 2. Read the 4 metadata floats (da, di, h, table_h)
        metadata_arr = np.fromfile(f, dtype=np.float32, count=4)
        # 3. Read the payload data
        payload = np.fromfile(f, dtype=np.float32, count=np.prod(shape)).reshape(shape)
  

    return [metadata_arr, payload]


def plot_eval(path_str, recon_file, scan_file):

    metadata_arr1, rec = load_recon_binary(recon_file)
    metadata_arr2, scan = load_scan_binary(scan_file)

    assert np.allclose(metadata_arr1, metadata_arr2), "Metadata mismatch between recon and scan files"

    da = float(metadata_arr1[0])
    di = float(metadata_arr1[1])
    h = float(metadata_arr1[2])
    table_h = float(metadata_arr1[3])

    scan_metrics_list = [
        "H_PLANE_FIT",           # yes base_fit[0][0]
        "ANGLE_MAJOR_LINE_FIT",  # yes angle_eval[0][0]
        "ANGLE_MINOR_LINE_FIT",  # yes angle_eval[0][1]
        "RADIUS_MAJOR_LINE_FIT", # yes radius_eval[0][1]
        "RADIUS_MINOR_LINE_FIT", # yes radius_eval[0][1]
        "RADIUS_CYL_FIT"         # yes cyl_fit_eval[0][0]
    ]

    rec_metrics_list = [
        "CENTER_CYL_FIT",        # yes cyl_fit_eval[0][1]
        "AXIS_CYL_FIT",          # yes cyl_fit_eval[0][2]
        "BASE_PARALLELISM"       # yes base_fit[0][1]
    ]

    table = {
        0: ["In", di, table_h + h],
        1: ["Out", da, table_h + h],
        2: ["Under", da, table_h]
    }

    # Subtract the real values from the scan metrics for plotting
    for loc in range(scan.shape[0]):
        scan[loc, :, :, 0] -= table[loc][2]  # H_PLANE_FIT
        scan[loc, :, :, 3] -= table[loc][1]  # RADIUS_MAJOR_LINE_FIT
        scan[loc, :, :, 4] -= table[loc][1]  # RADIUS_MINOR_LINE_FIT
        scan[loc, :, :, 5] -= table[loc][1]  # RADIUS_CYL_FIT
   
    lines_per_plot = 3
    for cali in range(scan.shape[1]):
        scan_cali = scan[:, cali, :, :].squeeze()
        rec_cali  = rec[cali, :, :].squeeze()
        fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(15, 15))
        for loc in range(scan.shape[0]):
            scan_loc = scan_cali[loc, :, :].squeeze()
            for metric_idx in range(len(scan_metrics_list)):
               col_num = metric_idx // lines_per_plot
               ax = axes[loc][col_num]
               ax.set_title(f'Scan Location {table[loc][0]}')
               ax.plot(scan_loc[:, metric_idx], label=f'{scan_metrics_list[metric_idx]}')
               ax.set_xlabel('Evaluation indices')
               ax.set_ylabel('error')
               ax.legend()
               ax.grid()
        for j in range(len(rec_metrics_list)):
            axes[0][2].set_title(f'Reconstruction metrics for cylinder ')
            axes[0][2].plot(rec_cali[:, j], label=f'{rec_metrics_list[j]}')
        axes[0][2].legend()
        axes[0][2].grid()
        plt.tight_layout()
        fig.savefig(os.path.join(path_str, f"eval_cali_{cali}.png"), dpi=100)
        plt.close()

    funcs = [np.mean, np.max]
    desc = ['average', 'maximum']
    scan = np.abs(scan)
    for func, desc_str in zip(funcs, desc):
        scan_cmp = func(scan, axis=2).squeeze()
        rec_cmp = func(rec, axis=1).squeeze()
        fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5))
        for loc in range(scan.shape[0]):
            for metric_idx in range(len(scan_metrics_list)):
                axes[loc].plot(scan_cmp[loc, :, metric_idx], label=f'{scan_metrics_list[metric_idx]}')
            for metric_idx in range(len(rec_metrics_list)):
                axes[loc].plot(rec_cmp[:, metric_idx], label=f'{rec_metrics_list[metric_idx]}')
            axes[loc].set_xlabel('calibration indices')
            axes[loc].set_ylabel(f'{desc_str} error')
            axes[loc].set_title(f'Scan Location {table[loc][0]}')
            axes[loc].legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
            axes[loc].grid()
        plt.tight_layout()
        fig.savefig(os.path.join(path_str, f"cali_cmp_{desc_str}.png"), dpi=100)
        plt.close()

def main():
    if len(sys.argv) > 1:
        recon_file = "recon.bin"
        scan_file = "scan.bin"
        path = sys.argv[1]  # sys.argv[0] is the script name itself
        plot_eval(path, os.path.join(path, recon_file), os.path.join(path, scan_file))
    else:
        print("No argument provided!!!!\n Set the cali evaluation path as the first positional argument.")

if __name__ == "__main__":
    main()