import lldb # type: ignore
import gc
import os.path
import plotly.graph_objects as go
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import parse
import numpy as np

def plot_hist(show):
    import debugger # type: ignore
    frame = lldb.debugger.GetSelectedTarget().GetProcess().GetSelectedThread().GetSelectedFrame()
    if not frame.IsValid():
        print("DEBUG ERROR: Invalid stack frame!")
        return True # The c++ program stops here.

    # 1. Evaluate variables from the current C++ stack frame
    scan_loc_val = frame.EvaluateExpression("scan_loc")
    path_val = frame.EvaluateExpression("path")

    path_str = path_val.GetSummary().strip('"')
    scan_loc = scan_loc_val.GetObjectDescription() or scan_loc_val.GetValue()

    points = parse.parse_vector(frame, "y_coords")

    title = f"Point Cloud Y distribution line {scan_loc}"
    bins = 100
    plt.figure()
    plt.hist(points, bins=bins, color='blue', alpha=0.75)
    plt.grid(True)
    file_name = os.path.join(path_str, f"y_distribution_{scan_loc}.png") 
    print(f"Saving histogram to {file_name}")
    plt.savefig(file_name, dpi=300)
    plt.close()
    gc.collect()
    if show==1:
        fig = go.Figure(data=go.Histogram(
            x=points,
            nbinsx=bins,
            marker=dict(
            color='blue',
            opacity=0.75))
            )
        fig.update_layout(
            title=title,
            bargap=0.05,
            xaxis_title="Y Values",
            yaxis_title="Frequency of the base points"
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