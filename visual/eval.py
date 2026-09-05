import lldb # type: ignore
import debugger # type: ignore
import gc
import os.path
import plotly.graph_objects as go
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def plot_eval(da, di, h, table_h):

    da = float(da)
    di = float(di)
    h = float(h)
    table_h = float(table_h)

    scan_metrics_list = [
        "H_PLANE_FIT",           # yes base_fit[0][0]
        "ANGLE_MAJOR_LINE_FIT",  # yes angle_eval[0][0]
        "ANGLE_MINOR_LINE_FIT",  # yes angle_eval[0][1]
        "RADIUS_MAJOR_LINE_FIT", # yes radius_eval[0][1]
        "RADIUS_MINOR_LINE_FIT", # yes radius_eval[0][1]
        "RADIUS_CYL_FIT"         # yes cyl_fit_eval[0][0]
    ]

    # enum class ScanMetrics{
    #     // metrics for the bases
    #     H_PLANE_FIT,           // yes base_fit[0][0]
    #     // metrics for the lateral itself
    #     ANGLE_MAJOR_LINE_FIT,  // yes angle_eval[0][0]
    #     ANGLE_MINOR_LINE_FIT,  // yes angle_eval[0][1]
    #     RADIUS_MAJOR_LINE_FIT, // yes radius_eval[0][1]
    #     RADIUS_MINOR_LINE_FIT, // yes radius_eval[0][1]
    #     RADIUS_CYL_FIT,        // yes cyl_fit_eval[0][0]
    #     NUM
    #  };
    rec_metrics_list = [
        "CENTER_CYL_FIT",        # yes cyl_fit_eval[0][1]
        "AXIS_CYL_FIT",          # yes cyl_fit_eval[0][2]
        "BASE_PARALLELISM"       # yes base_fit[0][1]
    ]
    # enum class ReconMetrics{
    #     // metrics for the lateral
    #     CENTER_CYL_FIT,        // yes cyl_fit_eval[0][1]
    #     AXIS_CYL_FIT,          // yes cyl_fit_eval[0][2]
    #     // metrics for the bases
    #     BASE_PARALLELISM,      // yes base_fit[0][1]
    #     NUM
    # }
    scan_metrics = len(scan_metrics_list)
    rec_metrics  = len(rec_metrics_list)

    frame = lldb.debugger.GetSelectedTarget().GetProcess().GetSelectedThread().GetSelectedFrame()
    if not frame.IsValid():
        print("DEBUG ERROR: Invalid stack frame!")
        return True

    path_val = frame.EvaluateExpression("path_str")
    path_str = path_val.GetSummary().strip('"')

    dimensions = []
    for i in range(3):
        dim_val = frame.EvaluateExpression(f"eval_scan.shape()[{i}]")
        if dim_val.IsValid():
            dimensions.append(dim_val.GetValueAsUnsigned())
        else:
            print(f"DEBUG ERROR: Invalid dimension {i}!")
            return True
    dimensions.append(scan_metrics)

    data_ptr_val = frame.EvaluateExpression("eval_scan.origin()")
    if data_ptr_val.IsValid():
        # Get the memory address as an integer
        raw_address = data_ptr_val.GetValueAsUnsigned()
        # Read the raw bytes from memory
        address = raw_address
        total_bytes = int(4 * np.prod(dimensions))  # 4 bytes per float32
        process = frame.GetThread().GetProcess()
        error = lldb.SBError()
        memory = process.ReadMemory(address, total_bytes, error)
    else:
        print("DEBUG ERROR: Invalid data pointer!")
        return True
    scan = np.frombuffer(memory, dtype=np.float32).reshape(dimensions)

    data_ptr_val = frame.EvaluateExpression("eval_recon.origin()")
    rec_dimensions = dimensions[1:]
    if data_ptr_val.IsValid():
        # Get the memory address as an integer
        raw_address = data_ptr_val.GetValueAsUnsigned()
        # Read the raw bytes from memory
        address = raw_address
        total_bytes = int(4 * np.prod(rec_dimensions))  # 4 bytes per float32
        process = frame.GetThread().GetProcess()
        error = lldb.SBError()
        memory = process.ReadMemory(address, total_bytes, error)
    else:
        print("DEBUG ERROR: Invalid data pointer!")
        return True
    rec = np.frombuffer(memory, dtype=np.float32).reshape(rec_dimensions)

    # enum class ScanLocation
    # {
    #     In,
    #     Out,
    #     Under,
    #     NUM
    # };
    table = {
        0: ["In", di, table_h + h],
        1: ["Out", di, table_h + h],
        2: ["Under", da, table_h]
    }

    for cali in range(dimensions[1]):
        scan_cali = scan[:, cali, :, :].squeeze()
        rec_cali  = rec[cali, :, :].squeeze()
        fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(15, 15))
        for loc in range(dimensions[0]):
            scan_loc = scan_cali[loc, :, :].squeeze()
            axes[loc][0].set_title(f'Scan Location {table[loc][0]}')
            axes[loc][0].plot(scan_loc[:, 0] - table[loc][2], label='Height plane')
            axes[loc][0].plot(scan_loc[:, 1], label='Angle major line')
            axes[loc][0].plot(scan_loc[:, 2], label='Angle minor line')
            axes[loc][0].legend()
            axes[loc][0].grid()
            axes[loc][1].set_title(f'Scan Location {table[loc][0]}')
            axes[loc][1].plot(scan_loc[:, 3] - table[loc][1], label='Radius major line')
            axes[loc][1].plot(scan_loc[:, 4] - table[loc][1], label='Radius minor line')
            axes[loc][1].plot(scan_loc[:, 5] - table[loc][1], label='Radius cylinder')
            axes[loc][1].legend()
            axes[loc][1].grid()
        axes[0][2].set_title(f'Reconstruction metrics for cylinder ')
        axes[0][2].plot(rec_cali[:, 0], label='Center alignment')
        axes[0][2].plot(rec_cali[:, 1], label='Axis parallelism in degrees')
        axes[0][2].plot(rec_cali[:, 2], label='Base parallelism in degrees')
        axes[0][2].legend()
        axes[0][2].grid()
        plt.tight_layout()
        fig.savefig(os.path.join(path_str, f"eval_cali_{cali}.png"), dpi=600)
        plt.close()

    funcs = [np.mean, np.max]
    desc = ['average', 'maximum']
    for func, desc_str in zip(funcs, desc):
        scan_cmp = func(scan, axis=2).squeeze()
        rec_cmp = func(rec, axis=1).squeeze()
        fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5))
        for loc in range(dimensions[0]):
            for metric_idx in range(len(scan_metrics_list)):
                axes[loc].plot(scan_cmp[loc, :, metric_idx], label=f'{scan_metrics_list[metric_idx]}')
            for metric_idx in range(len(rec_metrics_list)):
                axes[loc].plot(rec_cmp[:, metric_idx], label=f'{rec_metrics_list[metric_idx]}')
            axes[loc].set_xlabel('calibration indices')
            axes[loc].set_ylabel(f'{desc_str} error')
            axes[loc].set_title(f'Scan Location {table[loc][0]}')
            axes[loc].legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
            axes[loc].grid()
        fig.subplots_adjust(bottom=0.2)
        plt.tight_layout()
        fig.savefig(os.path.join(path_str, f"cali_cmp_{desc_str}.png"), dpi=600)
        plt.close()

    return False