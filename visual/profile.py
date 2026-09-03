import lldb # type: ignore
import gc
import os.path
import plotly.graph_objects as go
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import parse
from parse import Point2D
import numpy as np

def plot_profile(show):
    import debugger # type: ignore
    frame = lldb.debugger.GetSelectedTarget().GetProcess().GetSelectedThread().GetSelectedFrame()
    if not frame.IsValid():
        print("DEBUG ERROR: Invalid stack frame!")
        return

    # 1. Evaluate variables from the current C++ stack frame
    scan_loc_val = frame.EvaluateExpression("scan_loc")
    path_val = frame.EvaluateExpression("path")

    path_str = path_val.GetSummary().strip('"')
    scan_loc = scan_loc_val.GetObjectDescription() or scan_loc_val.GetValue()
    tmp = parse.parse_vector(frame, "points", dimensions=[2])
    x, y = tmp[:, 0], tmp[:, 1]
    tmp = parse.parse_vector(frame, "points_major", dimensions=[2])
    xm, ym = tmp[:, 0], tmp[:, 1]

    line = parse.parse_cv_vec4(frame, "line")
    d = Point2D(line[0], line[1])
    p = Point2D(line[2], line[3])
    line_major = parse.parse_cv_vec4(frame, "line_major")
    dm = Point2D(line_major[0], line_major[1])
    pm = Point2D(line_major[2], line_major[3])

    title = f"{scan_loc} - Angle Estimation"

    plt.figure()
    plt.scatter(x, y, s=1.0, c='blue', marker='o')
    plt.axline(p.to_list(), (p+d).to_list(), color='red', linewidth=0.5, linestyle='-')
    plt.scatter(xm, ym, s=1.0, c='green', marker='s')
    plt.axline(pm.to_list(), (pm+dm).to_list(), color='orange', linewidth=0.5, linestyle='-')
    plt.title(title)
    plt.legend(['Minor Profile Points', 'Minor Fitted Line', 'Major Profile Points', 'Major Fitted Line'], loc='upper right')
    plt.grid(True)
    file_name = os.path.join(path_str, f"{scan_loc}_angle_estimation.png")
    print(f"Saving profile plot to {file_name}")
    plt.savefig(file_name, dpi=300)
    plt.close()
    gc.collect()
    if show==1:
        min_x = min(min(x), min(xm))
        max_x = max(max(x), max(xm))
        distance = max_x - min_x
        min_x -= 0.05 * distance
        max_x += 0.05 * distance
        min_y = min(min(y), min(ym))
        max_y = max(max(y), max(ym))
        distance_y = max_y - min_y
        min_y -= 0.05 * distance_y
        max_y += 0.05 * distance_y
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=x, 
            y=y, 
            mode='markers',
            marker=dict(size=5, color='blue'),
            name='Minor Profile Points'
        ))
        fig.add_trace(go.Scatter(
            x=[min_x, max_x],
            y=[p.y-d.y/d.x*(p.x-min_x), p.y-d.y/d.x*(p.x-max_x)],
            mode='lines',
            line=dict(color='red', width=1, dash='solid'),
            name='Minor Fitted Line'
        ))
        fig.add_trace(go.Scatter(
            x=xm, 
            y=ym, 
            mode='markers',
            marker=dict(size=5, color='green'),
            name='Major Profile Points'
        ))
        fig.add_trace(go.Scatter(
            x=[min_x, max_x],
            y=[pm.y-dm.y/dm.x*(pm.x-min_x), pm.y-dm.y/dm.x*(pm.x-max_x)],
            mode='lines',
            line=dict(color='orange', width=1, dash='solid'),
            name='Major Fitted Line'
        ))
        fig.update_layout(
            title=title,
            xaxis_title="X",
            yaxis_title="Y",
            xaxis=dict(range=[min_x, max_x]),
            yaxis=dict(range=[min_y, max_y]),
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
        # set  position to 2 to make it appear in the second tab of the debugger's UI
        debugger.display_html(html_content, title=title, position=2, reveal=True)
        del fig
        gc.collect()
    return False # The c++ program stops here.