#!/usr/bin/env python3
"""industrial cockpit for esp32 suspension

real-time scada interface
"""

import argparse
import json
import math
import threading
import time
from collections import deque

import dash
from dash import html, dcc
from dash.dependencies import Input, Output
import plotly.graph_objects as go
import websocket

# telem state
MAX_HISTORY = 400

telem = {
    "r": 0.0, "p": 0.0, "m": 0.0, "s": 0.0,
    "md": 0, "ua": 1500, "ub": 1500, "ax": 0,
}
history = {
    "t": deque(maxlen=MAX_HISTORY),
    "roll": deque(maxlen=MAX_HISTORY),
    "pitch": deque(maxlen=MAX_HISTORY),
    "motor": deque(maxlen=MAX_HISTORY),
    "steer": deque(maxlen=MAX_HISTORY),
    "us_a": deque(maxlen=MAX_HISTORY),
    "us_b": deque(maxlen=MAX_HISTORY),
}
ws_connected = False
msg_count = 0
t_start = time.time()


def ws_thread(url):
    global telem, ws_connected
    while True:
        try:
            ws = websocket.WebSocketApp(
                url,
                on_message=lambda _, msg: _on_msg(msg),
                on_open=lambda _: _set_conn(True),
                on_close=lambda *_: _set_conn(False),
                on_error=lambda _, e: _set_conn(False),
            )
            ws.run_forever(ping_interval=5)
        except Exception:
            pass
        ws_connected = False
        time.sleep(1)


def _set_conn(v):
    global ws_connected
    ws_connected = v


def _on_msg(msg):
    global telem, msg_count
    try:
        d = json.loads(msg)
        telem = d
        msg_count += 1
        now = time.time() - t_start
        history["t"].append(now)
        history["roll"].append(d.get("r", 0))
        history["pitch"].append(d.get("p", 0))
        history["motor"].append(d.get("m", 0))
        history["steer"].append(d.get("s", 0))
        history["us_a"].append(d.get("ua", 1500))
        history["us_b"].append(d.get("ub", 1500))
    except Exception:
        pass


# design system
BG = "#050a12"
PANEL = "#0c1220"
PANEL_BORDER = "#1a2744"
ACCENT = "#00e676"
ACCENT_DIM = "#004d2a"
BLUE = "#448aff"
BLUE_DIM = "#0d2b6b"
AMBER = "#ffab00"
AMBER_DIM = "#4a3200"
RED = "#ff1744"
RED_DIM = "#4a0011"
WHITE = "#e8eaf6"
MUTED = "#607d8b"
GRID = "#111e33"
FONT = "Inter, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, monospace"
MONO = "JetBrains Mono, 'SF Mono', 'Fira Code', Consolas, monospace"

GRAPH_CONFIG = {"displayModeBar": False, "staticPlot": False}
BASE_LAYOUT = dict(
    paper_bgcolor="rgba(0,0,0,0)",
    plot_bgcolor="rgba(0,0,0,0)",
    font=dict(color=MUTED, family=MONO, size=11),
    margin=dict(l=8, r=8, t=8, b=8),
    dragmode=False,
)


def _panel(children, **extra):
    style = {
        "backgroundColor": PANEL,
        "border": f"1px solid {PANEL_BORDER}",
        "borderRadius": "6px",
        "padding": "8px 10px",
        "position": "relative",
        "overflow": "hidden",
    }
    style.update(extra)
    return html.Div(children, style=style)


def _label(text, color=MUTED):
    return html.Div(text, style={
        "fontSize": "10px", "fontWeight": 600, "letterSpacing": "2px",
        "textTransform": "uppercase", "color": color,
        "fontFamily": MONO, "marginBottom": "4px",
    })


def _readout(id_str, suffix="", color=WHITE):
    return html.Span(id=id_str, style={
        "fontSize": "28px", "fontWeight": 700, "color": color,
        "fontFamily": MONO, "lineHeight": "1",
    })


# instrument builders

def make_attitude(roll, pitch):
    """artificial horizon"""
    fig = go.Figure()
    r_rad = math.radians(-roll)
    p_norm = max(-1.0, min(1.0, pitch / 45.0))

    # sky/ground
    cos_r, sin_r = math.cos(r_rad), math.sin(r_rad)
    for layer_y in [i * 0.05 for i in range(-20, 21)]:
        shifted = layer_y + p_norm * 0.5
        rx0 = -1.2 * cos_r - shifted * sin_r
        ry0 = -1.2 * sin_r + shifted * cos_r
        rx1 = 1.2 * cos_r - shifted * sin_r
        ry1 = 1.2 * sin_r + shifted * cos_r
        c = "#0d2240" if layer_y >= 0 else "#3e2614"
        fig.add_shape(type="line", x0=rx0, y0=ry0, x1=rx1, y1=ry1,
                      line=dict(color=c, width=8), layer="below")

    # horizon
    h_len = 0.85
    hx0 = -h_len * cos_r - p_norm * 0.5 * sin_r
    hy0 = -h_len * sin_r + p_norm * 0.5 * cos_r
    hx1 = h_len * cos_r - p_norm * 0.5 * sin_r
    hy1 = h_len * sin_r + p_norm * 0.5 * cos_r
    fig.add_shape(type="line", x0=hx0, y0=hy0, x1=hx1, y1=hy1,
                  line=dict(color=ACCENT, width=2.5))

    # pitch ladder
    for deg in [-20, -10, 10, 20]:
        p_off = deg / 45.0 * 0.5 + p_norm * 0.5
        w = 0.15 if abs(deg) == 10 else 0.25
        lx0 = -w * cos_r - p_off * sin_r
        ly0 = -w * sin_r + p_off * cos_r
        lx1 = w * cos_r - p_off * sin_r
        ly1 = w * sin_r + p_off * cos_r
        fig.add_shape(type="line", x0=lx0, y0=ly0, x1=lx1, y1=ly1,
                      line=dict(color="rgba(255,255,255,0.3)", width=1))
        fig.add_annotation(x=lx1 + 0.06 * cos_r, y=ly1 + 0.06 * sin_r,
                          text=f"{deg}", showarrow=False,
                          font=dict(size=8, color="rgba(255,255,255,0.35)"))

    # aircraft symbol
    for dx in [(-0.18, -0.06), (0.06, 0.18)]:
        fig.add_shape(type="line", x0=dx[0], y0=0, x1=dx[1], y1=0,
                      line=dict(color=AMBER, width=3))
    fig.add_shape(type="line", x0=0, y0=-0.04, x1=0, y1=0.04,
                  line=dict(color=AMBER, width=3))

    # roll pointer
    arc_r = 0.82
    for a in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
        a_rad = math.radians(a + 90)
        x = arc_r * math.cos(a_rad)
        y = arc_r * math.sin(a_rad)
        tick_len = 0.06 if a % 30 == 0 else 0.03
        x2 = (arc_r - tick_len) * math.cos(a_rad)
        y2 = (arc_r - tick_len) * math.sin(a_rad)
        fig.add_shape(type="line", x0=x, y0=y, x1=x2, y1=y2,
                      line=dict(color="rgba(255,255,255,0.4)", width=1))

    # roll pointer triangle
    ptr_a = math.radians(-roll + 90)
    px = (arc_r - 0.08) * math.cos(ptr_a)
    py = (arc_r - 0.08) * math.sin(ptr_a)
    fig.add_annotation(x=px, y=py, text="\u25BC", showarrow=False,
                      font=dict(size=12, color=ACCENT))

    # outer ring
    ring_pts = 60
    rx = [0.88 * math.cos(2 * math.pi * i / ring_pts) for i in range(ring_pts + 1)]
    ry = [0.88 * math.sin(2 * math.pi * i / ring_pts) for i in range(ring_pts + 1)]
    fig.add_trace(go.Scatter(x=rx, y=ry, mode="lines",
                             line=dict(color=PANEL_BORDER, width=2),
                             hoverinfo="skip", showlegend=False))

    fig.update_layout(
        **BASE_LAYOUT,
        height=310,
        xaxis=dict(range=[-1.05, 1.05], visible=False, fixedrange=True),
        yaxis=dict(range=[-1.05, 1.05], visible=False, fixedrange=True,
                   scaleanchor="x"),
    )
    return fig


def make_throttle(value):
    """throttle gauge"""
    pct = value * 100
    if value > 0.05:
        bar_color, step_colors = ACCENT, [
            dict(range=[0, 50], color=ACCENT_DIM),
            dict(range=[50, 80], color="#1a3a1a"),
            dict(range=[80, 100], color="#2a4a1a"),
        ]
    elif value < -0.05:
        bar_color, step_colors = RED, [
            dict(range=[-100, -50], color="#3a1a1a"),
            dict(range=[-50, 0], color=RED_DIM),
        ]
    else:
        bar_color, step_colors = MUTED, []

    fig = go.Figure(go.Indicator(
        mode="gauge+number",
        value=pct,
        number=dict(suffix="%", font=dict(size=26, color=WHITE, family=MONO)),
        gauge=dict(
            axis=dict(range=[-100, 100], tickwidth=1, tickcolor=GRID,
                      tickfont=dict(size=9, color=MUTED), dtick=25),
            bar=dict(color=bar_color, thickness=0.7),
            bgcolor="#0a0f1a",
            bordercolor=PANEL_BORDER, borderwidth=1,
            steps=step_colors,
            threshold=dict(line=dict(color=WHITE, width=2), thickness=0.75, value=0),
        ),
    ))
    fig.update_layout(**BASE_LAYOUT, height=230, margin=dict(l=30, r=30, t=40, b=15))
    return fig


def make_steering(value):
    """steering gauge"""
    deg = value * 45
    bar_color = BLUE if abs(value) > 0.05 else MUTED
    fig = go.Figure(go.Indicator(
        mode="gauge+number",
        value=deg,
        number=dict(suffix="\u00b0", font=dict(size=26, color=WHITE, family=MONO)),
        gauge=dict(
            axis=dict(range=[-45, 45], tickwidth=1, tickcolor=GRID,
                      tickfont=dict(size=9, color=MUTED), dtick=15),
            bar=dict(color=bar_color, thickness=0.7),
            bgcolor="#0a0f1a",
            bordercolor=PANEL_BORDER, borderwidth=1,
            steps=[
                dict(range=[-45, -15], color=BLUE_DIM),
                dict(range=[15, 45], color=BLUE_DIM),
            ],
            threshold=dict(line=dict(color=WHITE, width=2), thickness=0.75, value=0),
        ),
    ))
    fig.update_layout(**BASE_LAYOUT, height=230, margin=dict(l=30, r=30, t=40, b=15))
    return fig


def make_suspension(us_a, us_b):
    """suspension schematic"""
    fig = go.Figure()
    a_norm = (us_a - 1500) / 500.0
    b_norm = (us_b - 1500) / 500.0

    # chassis
    fig.add_shape(type="rect", x0=-0.7, x1=0.7, y0=0.05, y1=0.28,
                  fillcolor="#0f1a2e", line=dict(color=BLUE, width=1.5))
    fig.add_annotation(x=0, y=0.165, text="CHASSIS", showarrow=False,
                      font=dict(size=9, color=BLUE, family=MONO))

    for side, x_c, val, label in [("L", -0.45, a_norm, "CH-A"),
                                   ("R", 0.45, b_norm, "CH-B")]:
        travel = val * 0.12
        spring_top = 0.05
        spring_bot = spring_top - 0.22 - travel
        wheel_y = spring_bot - 0.08

        # spring
        n_coils = 5
        sp_h = spring_top - spring_bot
        xs, ys = [], []
        for i in range(n_coils * 2 + 1):
            frac = i / (n_coils * 2)
            yy = spring_top - frac * sp_h
            xx = x_c + (0.06 if i % 2 == 1 else -0.06 if i % 2 == 0 and i > 0 else 0)
            if i == 0 or i == n_coils * 2:
                xx = x_c
            xs.append(xx)
            ys.append(yy)
        spring_color = AMBER if abs(val) > 0.3 else ACCENT if abs(val) < 0.1 else MUTED
        fig.add_trace(go.Scatter(x=xs, y=ys, mode="lines",
                                 line=dict(color=spring_color, width=2.5),
                                 hoverinfo="skip", showlegend=False))

        # damper
        fig.add_shape(type="rect",
                      x0=x_c - 0.025, x1=x_c + 0.025,
                      y0=spring_bot, y1=spring_bot + sp_h * 0.3,
                      fillcolor="#1a2744", line=dict(color=MUTED, width=1))

        # wheel
        fig.add_shape(type="circle",
                      x0=x_c - 0.1, x1=x_c + 0.1,
                      y0=wheel_y - 0.06, y1=wheel_y + 0.06,
                      fillcolor="#1a1a2e", line=dict(color="#4a4a6a", width=2))
        fig.add_shape(type="circle",
                      x0=x_c - 0.03, x1=x_c + 0.03,
                      y0=wheel_y - 0.02, y1=wheel_y + 0.02,
                      fillcolor="#2a2a4e", line=dict(color="#6a6a8a", width=1))

        # readings
        us_val = us_a if side == "L" else us_b
        fig.add_annotation(x=x_c, y=0.38, text=label, showarrow=False,
                          font=dict(size=9, color=MUTED, family=MONO))
        fig.add_annotation(x=x_c, y=0.44, text=f"{us_val}\u00b5s", showarrow=False,
                          font=dict(size=11, color=AMBER, family=MONO))

    # ground
    fig.add_shape(type="line", x0=-0.9, y0=-0.42, x1=0.9, y1=-0.42,
                  line=dict(color=GRID, width=1, dash="dot"))

    fig.update_layout(
        **BASE_LAYOUT, height=310,
        xaxis=dict(range=[-1, 1], visible=False, fixedrange=True),
        yaxis=dict(range=[-0.5, 0.52], visible=False, fixedrange=True,
                   scaleanchor="x"),
    )
    return fig


def make_timeseries():
    """strip chart"""
    t = list(history["t"])
    if not t:
        t = [0]

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=t, y=list(history["roll"]), name="Roll",
        line=dict(color=ACCENT, width=1.5), fill="tozeroy",
        fillcolor="rgba(0,230,118,0.05)"))
    fig.add_trace(go.Scatter(
        x=t, y=list(history["pitch"]), name="Pitch",
        line=dict(color=BLUE, width=1.5), fill="tozeroy",
        fillcolor="rgba(68,138,255,0.05)"))
    fig.add_trace(go.Scatter(
        x=t, y=[v * 100 for v in history["motor"]], name="Motor %",
        line=dict(color=AMBER, width=1, dash="dot")))
    fig.add_trace(go.Scatter(
        x=t, y=[v * 45 for v in history["steer"]], name="Steer \u00b0",
        line=dict(color="#7c4dff", width=1, dash="dashdot")))

    t_max = t[-1] if t else 0
    t_min = max(0, t_max - 15)

    fig.update_layout(
        **BASE_LAYOUT,
        height=195,
        margin=dict(l=45, r=10, t=8, b=30),
        xaxis=dict(
            gridcolor=GRID, gridwidth=1, title=None,
            range=[t_min, t_max],
            tickfont=dict(size=9, color=MUTED),
            ticksuffix="s",
        ),
        yaxis=dict(
            gridcolor=GRID, gridwidth=1, title=None,
            tickfont=dict(size=9, color=MUTED),
            zeroline=True, zerolinecolor="#1a2744", zerolinewidth=1,
        ),
        legend=dict(
            orientation="h", y=1.12, x=0.5, xanchor="center",
            font=dict(size=9, color=MUTED),
            bgcolor="rgba(0,0,0,0)",
        ),
        hovermode="x unified",
        hoverlabel=dict(bgcolor=PANEL, font=dict(size=10, family=MONO)),
    )
    return fig


# dash app
app = dash.Dash(__name__, update_title=None)
app.title = "Active Suspension Telemetry"

app.index_string = '''<!DOCTYPE html>
<html>
<head>
    {%metas%}
    <title>{%title%}</title>
    {%favicon%}
    {%css%}
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { background: ''' + BG + '''; overflow-x: hidden; }
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: ''' + BG + '''; }
        ::-webkit-scrollbar-thumb { background: ''' + PANEL_BORDER + '''; border-radius: 3px; }
        .js-plotly-plot .plotly .modebar { display: none !important; }
    </style>
</head>
<body>
    {%app_entry%}
    <footer>{%config%}{%scripts%}{%renderer%}</footer>
</body>
</html>'''

app.layout = html.Div(
    style={
        "maxWidth": "1400px", "margin": "0 auto", "padding": "12px 16px",
        "fontFamily": FONT, "color": WHITE,
    },
    children=[
        # header
        html.Div(style={
            "display": "flex", "justifyContent": "space-between",
            "alignItems": "flex-end", "marginBottom": "14px",
            "borderBottom": f"1px solid {PANEL_BORDER}", "paddingBottom": "10px",
        }, children=[
            html.Div([
                html.Div("ACTIVE SUSPENSION PLATFORM", style={
                    "fontSize": "18px", "fontWeight": 700, "letterSpacing": "3px",
                    "color": WHITE, "fontFamily": MONO,
                }),
                html.Div("ESP32 \u00b7 BTS7960 \u00b7 PCA9685 \u00b7 MPU6050", style={
                    "fontSize": "10px", "color": MUTED, "letterSpacing": "1.5px",
                    "marginTop": "2px", "fontFamily": MONO,
                }),
            ]),
            html.Div(id="header-status", style={
                "display": "flex", "gap": "16px", "alignItems": "center",
            }),
        ]),

        # row 1
        html.Div(style={
            "display": "grid",
            "gridTemplateColumns": "1.2fr 1fr 1fr 1.2fr",
            "gap": "10px", "marginBottom": "10px",
        }, children=[
            # attitude
            _panel([
                _label("ATTITUDE INDICATOR"),
                dcc.Graph(id="attitude", config=GRAPH_CONFIG),
                html.Div(style={
                    "display": "flex", "justifyContent": "space-around",
                    "marginTop": "4px",
                }, children=[
                    html.Div([
                        html.Div("ROLL", style={"fontSize": "9px", "color": MUTED,
                                                 "fontFamily": MONO}),
                        html.Div(id="roll-val", style={
                            "fontSize": "20px", "fontWeight": 700, "color": ACCENT,
                            "fontFamily": MONO}),
                    ], style={"textAlign": "center"}),
                    html.Div([
                        html.Div("PITCH", style={"fontSize": "9px", "color": MUTED,
                                                  "fontFamily": MONO}),
                        html.Div(id="pitch-val", style={
                            "fontSize": "20px", "fontWeight": 700, "color": BLUE,
                            "fontFamily": MONO}),
                    ], style={"textAlign": "center"}),
                ]),
            ]),

            # throttle
            _panel([
                _label("MOTOR THROTTLE"),
                dcc.Graph(id="throttle", config=GRAPH_CONFIG),
                html.Div(id="motor-bar", style={
                    "height": "4px", "borderRadius": "2px",
                    "backgroundColor": GRID, "marginTop": "6px",
                    "position": "relative", "overflow": "hidden",
                }),
            ]),

            # steering
            _panel([
                _label("STEERING ANGLE"),
                dcc.Graph(id="steering", config=GRAPH_CONFIG),
            ]),

            # suspension
            _panel([
                _label("SUSPENSION SCHEMATIC"),
                dcc.Graph(id="suspension", config=GRAPH_CONFIG),
            ]),
        ]),

        # row 2
        html.Div(style={
            "display": "grid",
            "gridTemplateColumns": "1fr 1fr 1fr 1fr 1fr 1fr",
            "gap": "8px", "marginBottom": "10px",
        }, children=[
            _panel([
                _label("MODE"),
                html.Div(id="mode-val", style={
                    "fontSize": "16px", "fontWeight": 700, "fontFamily": MONO}),
            ]),
            _panel([
                _label("ACCEL X"),
                html.Div(id="ax-val", style={
                    "fontSize": "16px", "fontWeight": 600, "color": WHITE,
                    "fontFamily": MONO}),
            ]),
            _panel([
                _label("SUS CH-A"),
                html.Div(id="usa-val", style={
                    "fontSize": "16px", "fontWeight": 600, "color": AMBER,
                    "fontFamily": MONO}),
            ]),
            _panel([
                _label("SUS CH-B"),
                html.Div(id="usb-val", style={
                    "fontSize": "16px", "fontWeight": 600, "color": AMBER,
                    "fontFamily": MONO}),
            ]),
            _panel([
                _label("DATA RATE"),
                html.Div(id="rate-val", style={
                    "fontSize": "16px", "fontWeight": 600, "color": WHITE,
                    "fontFamily": MONO}),
            ]),
            _panel([
                _label("UPTIME"),
                html.Div(id="uptime-val", style={
                    "fontSize": "16px", "fontWeight": 600, "color": WHITE,
                    "fontFamily": MONO}),
            ]),
        ]),

        # row 3
        _panel([
            _label("TELEMETRY STRIP CHART"),
            dcc.Graph(id="timeseries", config=GRAPH_CONFIG),
        ]),

        # interval
        dcc.Interval(id="tick", interval=80, n_intervals=0),
    ],
)


# callbacks
@app.callback(
    [
        Output("attitude", "figure"),
        Output("throttle", "figure"),
        Output("steering", "figure"),
        Output("suspension", "figure"),
        Output("timeseries", "figure"),
        Output("roll-val", "children"),
        Output("pitch-val", "children"),
        Output("mode-val", "children"),
        Output("mode-val", "style"),
        Output("ax-val", "children"),
        Output("usa-val", "children"),
        Output("usb-val", "children"),
        Output("rate-val", "children"),
        Output("uptime-val", "children"),
        Output("header-status", "children"),
    ],
    Input("tick", "n_intervals"),
)
def update(n):
    d = telem
    roll = d.get("r", 0)
    pitch = d.get("p", 0)
    motor = d.get("m", 0)
    steer = d.get("s", 0)
    mode = d.get("md", 0)
    us_a = d.get("ua", 1500)
    us_b = d.get("ub", 1500)
    ax_val = d.get("ax", 0)

    elapsed = time.time() - t_start
    rate = msg_count / max(elapsed, 1)
    mins = int(elapsed // 60)
    secs = int(elapsed % 60)

    mode_text = "ACTIVE PID" if mode else "MANUAL"
    mode_color = ACCENT if mode else AMBER
    conn_color = ACCENT if ws_connected else RED

    mode_style = {
        "fontSize": "16px", "fontWeight": 700, "fontFamily": MONO,
        "color": mode_color,
    }

    header = [
        html.Div(style={
            "width": "8px", "height": "8px", "borderRadius": "50%",
            "backgroundColor": conn_color,
            "boxShadow": f"0 0 6px {conn_color}",
        }),
        html.Span(
            "LINK UP" if ws_connected else "NO LINK",
            style={"fontSize": "10px", "color": conn_color,
                   "fontFamily": MONO, "fontWeight": 600,
                   "letterSpacing": "1px"},
        ),
        html.Span(
            f"{rate:.0f} Hz",
            style={"fontSize": "10px", "color": MUTED,
                   "fontFamily": MONO, "letterSpacing": "1px"},
        ),
    ]

    return (
        make_attitude(roll, pitch),
        make_throttle(motor),
        make_steering(steer),
        make_suspension(us_a, us_b),
        make_timeseries(),
        f"{roll:+.1f}\u00b0",
        f"{pitch:+.1f}\u00b0",
        mode_text,
        mode_style,
        f"{ax_val}",
        f"{us_a} \u00b5s",
        f"{us_b} \u00b5s",
        f"{rate:.1f} Hz",
        f"{mins:02d}:{secs:02d}",
        header,
    )


# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws", default="ws://localhost:8765")
    parser.add_argument("--port", type=int, default=8050)
    args = parser.parse_args()

    t = threading.Thread(target=ws_thread, args=(args.ws,), daemon=True)
    t.start()
    print(f"Dashboard: http://localhost:{args.port}")
    print(f"Telemetry: {args.ws}")

    app.run(debug=False, host="0.0.0.0", port=args.port)


if __name__ == "__main__":
    main()
