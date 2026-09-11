import lldb # type: ignore
import debugger # type: ignore
import gc
import os.path
import plotly.graph_objects as go
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import parse

def plot_line(idx, show, ra, ri, h, table_h):
    frame = lldb.debugger.GetSelectedTarget().GetProcess().GetSelectedThread().GetSelectedFrame()
    if not frame.IsValid():
        print("DEBUG ERROR: Invalid stack frame!")
        return True # The c++ program stops here.

    path_val = frame.EvaluateExpression("path")
    path_str = path_val.GetSummary().strip('"')
    # debug only for verification

    pts_val = frame.EvaluateExpression("points")
    scan_loc_val = frame.EvaluateExpression("scan_loc")
    scan_loc = scan_loc_val.GetObjectDescription() or scan_loc_val.GetValue()

    # Because of the order of the scan, watch the video
    flag = 1 if "Out" in scan_loc else -1

    # align all the scan with out
    if not "Out" in scan_loc:
        idx = (idx + 180) % 360
    

    points = parse.parse_vector(frame, "points", dimensions=[2])
    x = points[:, 0] * flag
    y = points[:, 1]

    # 3. Map colors to match your C++ color logic
    color_map = {
        "In": "blue",
        "Out": "green",
        "Under": "red"
    }

    offset = 10.0
    x_lower = -ra - offset
    x_upper = -ri + offset
    y_lower = (table_h - offset) if "Under" in scan_loc else (table_h + h - offset)
    y_upper = (table_h + h + offset) if "Under" in scan_loc else (table_h + h + offset)
    title = f"{scan_loc} line {idx} reconstruction"

    plt.figure()
    plt.scatter(x, y, s=1.0, color=color_map.get(scan_loc, "gray"), marker='o')
    plt.title(title)
    plt.xlim([x_lower, x_upper])
    plt.ylim([y_lower, y_upper])
    plt.grid(True)
    file_name = f"line_{int(idx):03d}_{scan_loc}_recon.png"
    plt.savefig(os.path.join(path_str, file_name), dpi=300)
    plt.close()
    gc.collect()

    if show==1:
        fig = go.Figure(data=go.Scatter(
            x=x, 
            y=y, 
            mode='markers',
            marker=dict(size=5, color=color_map.get(scan_loc, "gray"))
        ))
        fig.update_layout(
            title=title,
            xaxis_title="X",
            yaxis_title="Y",
            # Enables automatic scaling for the X-axis
            xaxis=dict(autorange=True),
            yaxis=dict(autorange=True),
            # xaxis=dict(range=[x_lower, x_upper]),
            # yaxis=dict(range=[y_lower, y_upper]),
            dragmode='pan',
        )

        # 6. Convert to HTML and wrap in a 100% viewport container with injected CSS overrides
        raw_html = fig.to_html(include_plotlyjs='cdn', full_html=False, config={'responsive': True})

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                html, body {{
                    width: 100%;
                    height: 100%;
                    margin: 0;
                    padding: 0;
                    background-color: #1e1e1e;
                    overflow: hidden;
                }}
                #full-screen-plot {{
                    width: 100% !important;
                    height: 100% !important;
                }}
                /* Force Plotly internal SVG containers to obey 100% sizing */
                .plotly-graph-div, .svg-container {{
                    width: 100% !important;
                    height: 100% !important;
                }}
            </style>
        </head>
        <body>
            <div id="full-screen-plot">
                {raw_html}
            </div>
        </body>
        </html>
        """
        # set position to 2 to make it appear in the second tab of the debugger's UI
        debugger.display_html(html_content, title=f"Reconstruction - {scan_loc}", position=2, reveal=True)
        del fig
        gc.collect()
    return False # The c++ program stops here.