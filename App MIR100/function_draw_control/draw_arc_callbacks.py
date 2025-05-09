import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
from dash.exceptions import PreventUpdate
import numpy as np

def register_draw_arc_callbacks(app):
    # Hiển thị popup chọn chế độ vẽ cung
    @app.callback(
        Output("draw-control-popups", "children", allow_duplicate=True),
        Input("draw-arc-btn", "n_clicks"),
        State("draw-control-popups", "children"),
        prevent_initial_call=True
    )
    def show_draw_arc_popup(n_clicks, existing_popups):
        if n_clicks is None:
            return existing_popups

        popup = dbc.Modal(
            [
                dbc.ModalHeader(dbc.ModalTitle("Draw Arc Mode")),
                dbc.ModalBody(
                    [
                        dbc.RadioItems(
                            options=[
                                {"label": "Manual (Click 3 Points)", "value": "manual"},
                                {"label": "Enter Coordinates", "value": "coordinates"},
                            ],
                            value="manual",
                            id="draw-arc-mode-radio",
                            className="mb-3",
                        ),
                        html.Div(id="draw-arc-mode-content"),
                    ]
                ),
                dbc.ModalFooter(
                    [
                        dbc.Button("Close", id="close-draw-arc-modal", className="ms-auto"),
                    ]
                ),
            ],
            id="draw-arc-modal",
            is_open=True,
        )

        if existing_popups is None:
            return popup
        else:
            return existing_popups + [popup]

    # Cập nhật nội dung trong popup dựa trên chế độ chọn
    @app.callback(
        Output("draw-arc-mode-content", "children"),
        Input("draw-arc-mode-radio", "value")
    )
    def update_draw_arc_content(mode):
        if mode == "manual":
            return html.Div([
                html.P("Click 3 points on the graph to define the arc."),
                dbc.Button("OK", id="draw-arc-manual-ok", n_clicks=0, color="primary", className="mt-2")
            ])
        elif mode == "coordinates":
            return html.Div(
                [
                    dbc.Row(
                        [
                            dbc.Col([dbc.Label("Point 1 X"), dcc.Input(id="arc-p1-x", type="number", placeholder="X")]),
                            dbc.Col([dbc.Label("Point 1 Y"), dcc.Input(id="arc-p1-y", type="number", placeholder="Y")]),
                        ]
                    ),
                    dbc.Row(
                        [
                            dbc.Col([dbc.Label("Point 2 X"), dcc.Input(id="arc-p2-x", type="number", placeholder="X")]),
                            dbc.Col([dbc.Label("Point 2 Y"), dcc.Input(id="arc-p2-y", type="number", placeholder="Y")]),
                        ]
                    ),
                    dbc.Row(
                        [
                            dbc.Col([dbc.Label("Point 3 X"), dcc.Input(id="arc-p3-x", type="number", placeholder="X")]),
                            dbc.Col([dbc.Label("Point 3 Y"), dcc.Input(id="arc-p3-y", type="number", placeholder="Y")]),
                        ]
                    ),
                    dbc.Button("Draw Arc", id="draw-arc-coords-button", color="primary", className="mt-2"),
                    html.Div(id="draw-arc-coords-message", className="mt-2")
                ]
            )
        else:
            return ""

    # Đóng popup khi nhấn nút Close
    @app.callback(
        Output("draw-arc-modal", "is_open"),
        Input("close-draw-arc-modal", "n_clicks"),
        State("draw-arc-modal", "is_open"),
        prevent_initial_call=True
    )
    def close_draw_arc_modal(n_clicks, is_open):
        if n_clicks:
            return False
        return is_open

    # Vẽ cung bằng cách nhập tọa độ
    @app.callback(
        [Output("draw-arc-coords-message", "children"),
         Output("draw-control-graph", "figure", allow_duplicate=True),
         Output("draw-data-store", "data", allow_duplicate=True)],
        Input("draw-arc-coords-button", "n_clicks"),
        State("arc-p1-x", "value"),
        State("arc-p1-y", "value"),
        State("arc-p2-x", "value"),
        State("arc-p2-y", "value"),
        State("arc-p3-x", "value"),
        State("arc-p3-y", "value"),
        State("draw-control-graph", "figure"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def draw_arc_with_coordinates(n_clicks, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, figure, data):
        if n_clicks is None:
            raise PreventUpdate

        if not all([isinstance(val, (int, float)) for val in [p1_x, p1_y, p2_x, p2_y, p3_x, p3_y]]):
            return "Please enter valid numerical coordinates for all three points.", dash.no_update, dash.no_update

        try:
            center_x, center_y, radius = calc_circle_params(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y)
            new_arc = {
                'center_x': center_x, 'center_y': center_y, 'radius': radius,
                'p1_x': p1_x, 'p1_y': p1_y, 'p2_x': p2_x, 'p2_y': p2_y, 'p3_x': p3_x, 'p3_y': p3_y
            }
            data['arcs'].append(new_arc)

            fig = go.Figure(figure)
            arc_trace = draw_arc(center_x, center_y, radius, p1_x, p1_y, p3_x, p3_y, num_points=100)
            fig.add_trace(arc_trace)

            return (f"Arc drawn with center ({center_x:.2f}, {center_y:.2f}) and radius {radius:.2f}", 
                    fig, data)
        except ValueError as e:
            return str(e), dash.no_update, dash.no_update

    # Kích hoạt chế độ vẽ thủ công và đóng popup
    @app.callback(
        [Output("draw-control-graph", "config", allow_duplicate=True),
         Output("draw-control-graph", "figure", allow_duplicate=True),
         Output("draw-arc-modal", "is_open", allow_duplicate=True),
         Output("draw-control-popups", "children", allow_duplicate=True)],
        Input("draw-arc-manual-ok", "n_clicks"),
        State("draw-arc-mode-radio", "value"),
        State("draw-arc-modal", "is_open"),
        State("draw-control-graph", "figure"),
        State("draw-control-graph", "config"),
        prevent_initial_call=True
    )
    def enable_manual_draw_arc_and_close(n_clicks, draw_mode, is_open, fig, config):
        if draw_mode == "manual" and n_clicks > 0:
            fig = go.Figure(fig)
            # Thêm scatter ẩn để bắt sự kiện nhấp chuột
            x_hidden = np.linspace(0, 20, 21)
            y_hidden = np.linspace(0, 20, 21)
            x_grid, y_grid = np.meshgrid(x_hidden, y_hidden)
            fig.add_trace(go.Scatter(
                x=x_grid.flatten(), y=y_grid.flatten(),
                mode="markers",
                marker=dict(size=1, opacity=0),  # Điểm ẩn
                showlegend=False,
                name="hidden_grid"
            ))
            fig.update_layout(
                clickmode="event+select",  # Bật sự kiện nhấp chuột
                uirevision="manual-draw-arc"
            )
            config["modeBarButtonsToRemove"] = ["lasso2d", "select2d", "pan2d"]
            print(f"Manual mode enabled with config: {config}, layout: {fig.layout}")
            return config, fig, False, None
        return dash.no_update, dash.no_update, is_open, dash.no_update

    # Xử lý vẽ cung thủ công khi nhấp chuột (Thêm debug)
    @app.callback(
        [Output("draw-data-store", "data", allow_duplicate=True),
         Output("draw-control-graph", "figure", allow_duplicate=True)],
        Input("draw-control-graph", "clickData"),
        State("draw-data-store", "data"),
        State("draw-control-graph", "figure"),
        State("draw-arc-mode-radio", "value"),
        prevent_initial_call=True
    )
    def handle_manual_draw_arc(clickData, data, figure, draw_mode):
        print(f"Checking callback trigger: draw_mode={draw_mode}, clickData={clickData}")  # Debug
        if draw_mode != "manual" or clickData is None:
            print("Draw mode not manual or clickData is None, preventing update.")
            raise PreventUpdate

        try:
            point = clickData["points"][0]
            x, y = point["x"], point["y"]
            print(f"Clicked point: x={x}, y={y}")
            data["temp_points"].append({"x": x, "y": y})

            fig = go.Figure(figure)
            fig.data = [trace for trace in fig.data if trace.name != "selected_points"]
            x_points = [p["x"] for p in data["temp_points"]]
            y_points = [p["y"] for p in data["temp_points"]]

            fig.add_trace(go.Scatter(
                x=x_points, y=y_points, mode="markers",
                marker=dict(size=10, color="red"),
                name="selected_points"
            ))

            if len(data["temp_points"]) == 3:
                p1, p2, p3 = data["temp_points"]
                print(f"Three points selected: p1={p1}, p2={p2}, p3={p3}")
                center_x, center_y, radius = calc_circle_params(p1["x"], p1["y"], p2["x"], p2["y"], p3["x"], p3["y"])
                print(f"Calculated circle: center_x={center_x}, center_y={center_y}, radius={radius}")
                new_arc = {
                    "center_x": center_x, "center_y": center_y, "radius": radius,
                    "p1_x": p1["x"], "p1_y": p1["y"], "p2_x": p2["x"], "p2_y": p2["y"], "p3_x": p3["x"], "p3_y": p3["y"]
                }
                data["arcs"].append(new_arc)
                arc_trace = draw_arc(center_x, center_y, radius, p1["x"], p1["y"], p3["x"], p3["y"], num_points=100)
                fig.add_trace(arc_trace)
                data["temp_points"] = []
                print(f"Arc drawn with center ({center_x:.2f}, {center_y:.2f}), radius {radius:.2f}")

            return data, fig

        except Exception as e:
            print(f"An error occurred: {e}")
            return dash.no_update, dash.no_update

# Hàm tính toán tâm và bán kính của đường tròn qua 3 điểm
def calc_circle_params(x1, y1, x2, y2, x3, y3):
    A = x2 - x1
    B = y2 - y1
    C = x3 - x1
    D = y3 - y1
    E = A * (x1 + x2) + B * (y1 + y2)
    F = C * (x1 + x3) + D * (y1 + y3)
    G = 2 * (A * (y3 - y2) - B * (x3 - x2))

    if G == 0:
        raise ValueError("Các điểm thẳng hàng, không thể vẽ cung tròn.")

    center_x = (D * E - B * F) / G
    center_y = (A * F - C * E) / G
    radius = np.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
    return center_x, center_y, radius

# Hàm vẽ cung từ điểm bắt đầu đến điểm kết thúc
def draw_arc(center_x, center_y, radius, start_x, start_y, end_x, end_y, num_points=50):
    start_angle = np.arctan2(start_y - center_y, start_x - center_x)
    end_angle = np.arctan2(end_y - center_y, end_x - center_x)
    
    if end_angle < start_angle:
        end_angle += 2 * np.pi
    if end_angle - start_angle > np.pi:
        end_angle -= 2 * np.pi

    angles = np.linspace(start_angle, end_angle, num_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return go.Scatter(x=x, y=y, mode="lines", line=dict(color="green", width=3))