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


def plot_eval(path_cali, path_eval, recon_file, scan_file):

    cali_root = os.path.dirname(os.path.normpath(path_cali))
    cali_config = os.path.join(cali_root, "config")
    cali_label = sorted([x for x in os.listdir(cali_config) if x.endswith(".ini")])
    cali_label = [x.removesuffix(".ini")[-4:] if len(x.removesuffix(".ini")) > 4 else x.removesuffix(".ini") for x in cali_label]

    eval_root = os.path.dirname(os.path.normpath(path_eval))
    eval_config = os.path.join(eval_root, "config")
    eval_label = sorted([x for x in os.listdir(eval_config) if x.endswith(".ini")])
    eval_label = [x.removesuffix(".ini")[-4:] if len(x.removesuffix(".ini")) > 4 else x.removesuffix(".ini") for x in eval_label]

    metadata_arr1, rec = load_recon_binary(recon_file)
    metadata_arr2, scan = load_scan_binary(scan_file)

    visual_dir = os.path.join(path_eval, "visual")
    os.makedirs(visual_dir, exist_ok=True)

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

    scan_metrics_style = [
        "-",                     # "H_PLANE_FIT",           
        "--",                    # "ANGLE_MAJOR_LINE_FIT",  
        "--",                    # "ANGLE_MINOR_LINE_FIT",  
        ":",                     # "RADIUS_MAJOR_LINE_FIT", 
        ":",                     # "RADIUS_MINOR_LINE_FIT", 
        "-",                     # "RADIUS_CYL_FIT"         
    ]

    rec_metrics_list = [
        "CENTER_CYL_FIT",        # yes cyl_fit_eval[0][1]
        "AXIS_CYL_FIT",          # yes cyl_fit_eval[0][2]
        "BASE_PARALLELISM"       # yes base_fit[0][1]
    ]

    rec_metrics_style = [
        "--",                    # "CENTER_CYL_FIT",
        "--",                    # "AXIS_CYL_FIT",
        "--"                     # "BASE_PARALLELISM"
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
        scan_cali = scan[:, cali, :, :]
        rec_cali  = rec[cali, :, :]
        fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(15, 15))
        for loc in range(scan.shape[0]):
            scan_loc = scan_cali[loc, :, :]
            for metric_idx in range(len(scan_metrics_list)):
               col_num = metric_idx // lines_per_plot
               ax = axes[loc][col_num]
               ax.set_title(f'Scan Location {table[loc][0]}')
               ax.plot(eval_label, scan_loc[:, metric_idx], 
                    label=f'{scan_metrics_list[metric_idx]}',
                    linestyle=scan_metrics_style[metric_idx])
               ax.set_xticks(range(len(eval_label)))
               ax.set_xticklabels(eval_label, rotation=90)
               ax.set_xlabel('Evaluation Ring Name')
               ax.set_ylabel('Error')
               ax.legend()
               ax.grid()
        for j in range(len(rec_metrics_list)):
            axes[0][2].set_title(f'Reconstruction metrics for cylinder ')
            axes[0][2].plot(eval_label, rec_cali[:, j], 
                            label=f'{rec_metrics_list[j]}',
                            linestyle=rec_metrics_style[j])
        axes[0][2].set_xticks(range(len(eval_label)))
        axes[0][2].set_xticklabels(eval_label, rotation=90)
        axes[0][2].set_xlabel('Evaluation Ring Name')
        axes[0][2].set_ylabel('Error')
        axes[0][2].legend()
        axes[0][2].grid()
        plt.tight_layout()
        fig.savefig(os.path.join(visual_dir, f"eval_cali_{cali_label[cali]}.png"), dpi=100)
        plt.close()

    funcs = [np.mean, np.max]
    desc = ['Average', 'Maximum']
    scan = np.abs(scan)
    for func, desc_str in zip(funcs, desc):
        scan_cmp = func(scan, axis=2)
        rec_cmp = func(rec, axis=1)
        fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)
        for loc in range(scan.shape[0]):
            for metric_idx in range(len(scan_metrics_list)):
                axes[loc].plot(cali_label, scan_cmp[loc, :, metric_idx],
                               linestyle=scan_metrics_style[metric_idx],
                               label=f'{scan_metrics_list[metric_idx]}')
            for metric_idx in range(len(rec_metrics_list)):
                axes[loc].plot(cali_label, rec_cmp[:, metric_idx],
                               linestyle=rec_metrics_style[metric_idx],
                               label=f'{rec_metrics_list[metric_idx]}')
            axes[loc].set_xlabel('Calibration Ring Name')
            axes[loc].set_xticks(range(len(cali_label)))
            axes[loc].set_xticklabels(cali_label, rotation=90)
            axes[loc].set_title(f'Scan Location {table[loc][0]}')
            axes[loc].grid()
        fig.text(
            0.08,
            0.55,
            f"{desc_str} Error",
            va="center",
            ha="center",
            rotation="vertical",
            fontsize=12,
        )
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=2)
        plt.subplots_adjust(bottom=0.10, wspace=0.05)
        fig.savefig(os.path.join(visual_dir, f"cali_cmp_{desc_str.lower()}.png"), dpi=100, bbox_inches="tight")
        plt.close()

def main():
    if len(sys.argv) > 1:
        recon_file = "recon.bin"
        scan_file = "scan.bin"
        path_eval = sys.argv[1]  # sys.argv[0] is the script name itself
        rec = os.path.join(path_eval, recon_file)
        scan = os.path.join(path_eval, scan_file)
        if len(sys.argv) > 2:
            path_cali = sys.argv[2]
            plot_eval(path_cali, path_eval, rec, scan)
        else:
            plot_eval(path_eval, path_eval, rec, scan)
    else:
        print("No argument provided!!!!\n Set the cali evaluation path as the first positional argument.")

if __name__ == "__main__":
    main()