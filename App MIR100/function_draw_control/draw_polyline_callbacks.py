import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
from dash.exceptions import PreventUpdate
import numpy as np

def register_draw_polyline_callbacks(app):
    # Hiển thị popup chọn chế độ vẽ polyline
    @app.callback(
        Output("draw-control-popups", "children", allow_duplicate=True),
        Input("draw-polyline-btn", "n_clicks"),
        State("draw-control-popups", "children"),
        prevent_initial_call=True
    )
    def show_draw_polyline_popup(n_clicks, existing_popups):
        if n_clicks is None:
            return existing_popups

        popup = dbc.Modal(
            [
                dbc.ModalHeader(dbc.ModalTitle("Draw Polyline Mode")),
                dbc.ModalBody(
                    [
                        dbc.RadioItems(
                            options=[
                                {"label": "Manual (Click Points)", "value": "manual"},
                                {"label": "Enter Coordinates", "value": "coordinates"},
                            ],
                            value="manual",
                            id="draw-polyline-mode-radio",
                            className="mb-3",
                        ),
                        html.Div(id="draw-polyline-mode-content"),
                    ]
                ),
                dbc.ModalFooter(
                    [
                        dbc.Button("Close", id="close-draw-polyline-modal", className="ms-auto"),
                    ]
                ),
            ],
            id="draw-polyline-modal",
            is_open=True,
        )

        if existing_popups is None:
            return popup
        else:
            return existing_popups + [popup]

    # Cập nhật nội dung trong popup dựa trên chế độ chọn
    @app.callback(
        Output("draw-polyline-mode-content", "children"),
        Input("draw-polyline-mode-radio", "value")
    )
    def update_draw_polyline_content(mode):
        if mode == "manual":
            return html.Div([
                html.P("Click points on the graph to draw a polyline (double-click to finish)."),
                dbc.Button("OK", id="draw-polyline-manual-ok", n_clicks=0, color="primary", className="mt-2")
            ])
        elif mode == "coordinates":
            return html.Div([
                html.P("Enter coordinates for each point, separated by commas (e.g., 'x1,y1, x2,y2, ...')."),
                dcc.Input(id="polyline-coords-input", type="text", placeholder="e.g., 5,5, 10,10, 15,5", className="mb-3"),
                dbc.Button("Draw Polyline", id="draw-polyline-coords-button", color="primary", className="mt-2"),
                html.Div(id="draw-polyline-coords-message", className="mt-2")
            ])
        else:
            return ""

    # Đóng popup khi nhấn nút Close
    @app.callback(
        Output("draw-polyline-modal", "is_open"),
        Input("close-draw-polyline-modal", "n_clicks"),
        State("draw-polyline-modal", "is_open"),
        prevent_initial_call=True
    )
    def close_draw_polyline_modal(n_clicks, is_open):
        if n_clicks:
            return False
        return is_open

    # Vẽ polyline bằng cách nhập tọa độ
    @app.callback(
        [Output("draw-polyline-coords-message", "children"),
         Output("draw-control-graph", "figure", allow_duplicate=True),
         Output("draw-data-store", "data", allow_duplicate=True)],
        Input("draw-polyline-coords-button", "n_clicks"),
        State("polyline-coords-input", "value"),
        State("draw-control-graph", "figure"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def draw_polyline_with_coordinates(n_clicks, coords_input, figure, data):
        if n_clicks is None or not coords_input:
            raise PreventUpdate

        try:
            # Phân tích chuỗi tọa độ (ví dụ: "5,5, 10,10, 15,5")
            coords = [float(x) for pair in coords_input.split() for x in pair.split(',')]
            if len(coords) % 2 != 0:
                return "Invalid coordinates format. Use 'x1,y1, x2,y2, ...'.", dash.no_update, dash.no_update

            points = [(coords[i], coords[i + 1]) for i in range(0, len(coords), 2)]
            new_polyline = {"points": points}
            data["polylines"].append(new_polyline)

            fig = go.Figure(figure)
            x_points, y_points = zip(*points)
            fig.add_trace(go.Scatter(
                x=list(x_points), y=list(y_points),
                mode="lines+markers",
                line=dict(color="blue", width=3),
                marker=dict(size=8, color="blue"),
                name="polyline"
            ))

            return (f"Polyline drawn with {len(points)} points.", fig, data)
        except ValueError as e:
            return f"Error: {str(e)}. Use format 'x1,y1, x2,y2, ...'.", dash.no_update, dash.no_update

    # Kích hoạt chế độ vẽ polyline thủ công và đóng popup
    @app.callback(
        [Output("draw-control-graph", "config", allow_duplicate=True),
         Output("draw-control-graph", "figure", allow_duplicate=True),
         Output("draw-polyline-modal", "is_open", allow_duplicate=True),
         Output("draw-control-popups", "children", allow_duplicate=True)],
        Input("draw-polyline-manual-ok", "n_clicks"),
        State("draw-polyline-mode-radio", "value"),
        State("draw-polyline-modal", "is_open"),
        State("draw-control-graph", "figure"),
        State("draw-control-graph", "config"),
        prevent_initial_call=True
    )
    def enable_manual_draw_polyline_and_close(n_clicks, draw_mode, is_open, fig, config):
        if draw_mode == "manual" and n_clicks > 0:
            fig = go.Figure(fig)
            # Thêm scatter ẩn để bắt sự kiện nhấp chuột trên toàn biểu đồ
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
                uirevision="manual-draw-polyline"
            )
            config["modeBarButtonsToRemove"] = ["lasso2d", "select2d", "pan2d"]
            print(f"Manual polyline mode enabled with config: {config}, layout: {fig.layout}")
            return config, fig, False, None
        return dash.no_update, dash.no_update, is_open, dash.no_update

    # Xử lý vẽ polyline thủ công khi nhấp chuột
    @app.callback(
        [Output("draw-data-store", "data", allow_duplicate=True),
         Output("draw-control-graph", "figure", allow_duplicate=True)],
        Input("draw-control-graph", "clickData"),
        State("draw-data-store", "data"),
        State("draw-control-graph", "figure"),
        State("draw-polyline-mode-radio", "value"),
        prevent_initial_call=True
    )
    def handle_manual_draw_polyline(clickData, data, figure, draw_mode):
        print(f"Checking polyline callback trigger: draw_mode={draw_mode}, clickData={clickData}")
        if draw_mode != "manual" or clickData is None:
            print("Polyline mode not manual or clickData is None, preventing update.")
            raise PreventUpdate

        try:
            point = clickData["points"][0]
            x, y = point["x"], point["y"]
            print(f"Clicked point for polyline: x={x}, y={y}")
            data["temp_points"].append({"x": x, "y": y})

            fig = go.Figure(figure)
            # Giữ scatter ẩn và chỉ cập nhật polyline
            fig.data = [trace for trace in fig.data if trace.name not in ["polyline", "selected_points"]]
            x_points = [p["x"] for p in data["temp_points"]]
            y_points = [p["y"] for p in data["temp_points"]]

            # Hiển thị polyline tạm thời với các điểm đã chọn
            fig.add_trace(go.Scatter(
                x=x_points, y=y_points, mode="lines+markers",
                line=dict(color="blue", width=3),
                marker=dict(size=8, color="blue"),
                name="polyline"
            ))

            # Kết thúc polyline khi double-click (giả định double-click gửi lại clickData)
            if len(data["temp_points"]) >= 2:  # Ít nhất 2 điểm để vẽ polyline
                new_polyline = {"points": [(p["x"], p["y"]) for p in data["temp_points"]]}
                data["polylines"].append(new_polyline)
                data["temp_points"] = []  # Reset sau khi hoàn thành
                print(f"Polyline drawn with {len(new_polyline['points'])} points.")

            return data, fig

        except Exception as e:
            print(f"An error occurred in polyline drawing: {e}")
            return dash.no_update, dash.no_update

    # Xử lý double-click để kết thúc polyline (giả định double-click gửi clickData)
    @app.callback(
        [Output("draw-data-store", "data", allow_duplicate=True),
         Output("draw-control-graph", "figure", allow_duplicate=True)],
        Input("draw-control-graph", "doubleClick"),
        State("draw-data-store", "data"),
        State("draw-control-graph", "figure"),
        State("draw-polyline-mode-radio", "value"),
        prevent_initial_call=True
    )
    def handle_double_click_end_polyline(double_click_data, data, figure, draw_mode):
        print(f"Double-click detected: draw_mode={draw_mode}, double_click_data={double_click_data}")
        if draw_mode != "manual" or double_click_data is None or not data["temp_points"]:
            raise PreventUpdate

        try:
            if len(data["temp_points"]) >= 2:
                new_polyline = {"points": [(p["x"], p["y"]) for p in data["temp_points"]]}
                data["polylines"].append(new_polyline)
                data["temp_points"] = []  # Reset sau khi hoàn thành
                print(f"Polyline finalized with {len(new_polyline['points'])} points.")

                fig = go.Figure(figure)
                fig.data = [trace for trace in fig.data if trace.name not in ["polyline", "selected_points"]]
                x_points, y_points = zip(*new_polyline["points"])
                fig.add_trace(go.Scatter(
                    x=list(x_points), y=list(y_points),
                    mode="lines+markers",
                    line=dict(color="blue", width=3),
                    marker=dict(size=8, color="blue"),
                    name="polyline"
                ))
                fig.update_layout(clickmode="event+select", dragmode="zoom")  # Trở lại chế độ mặc định
                return data, fig
            return dash.no_update, dash.no_update

        except Exception as e:
            print(f"Error finalizing polyline: {e}")
            return dash.no_update, dash.no_update

    # Nút Clear Polyline
    @app.callback(
        [Output("draw-control-graph", "figure", allow_duplicate=True),
         Output("draw-data-store", "data", allow_duplicate=True)],
        Input("clear-line-btn", "n_clicks"),
        State("draw-control-graph", "figure"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def clear_polyline(n_clicks, figure, data):
        if n_clicks is None:
            raise PreventUpdate

        fig = go.Figure(figure)
        fig.data = [trace for trace in fig.data if trace.name not in ["polyline", "selected_points"]]
        data["polylines"] = []
        data["temp_points"] = []
        return fig, data

    # Nút Save Polyline (giả định lưu vào file hoặc store)
    @app.callback(
        [Output("draw-control-popups", "children", allow_duplicate=True),
         Output("draw-data-store", "data", allow_duplicate=True)],
        Input("save-line-btn", "n_clicks"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def save_polyline(n_clicks, data):
        if n_clicks is None:
            raise PreventUpdate

        if not data["polylines"]:
            popup = dbc.Toast(
                "No polylines to save.",
                header="Save Polyline",
                duration=3000,
                is_open=True,
                style={"position": "fixed", "top": 10, "right": 10}
            )
            return popup, data

        # Giả định lưu vào file hoặc store (ở đây chỉ in ra console)
        print(f"Saving polylines: {data['polylines']}")
        popup = dbc.Toast(
            f"Saved {len(data['polylines'])} polylines.",
            header="Save Polyline",
            duration=3000,
            is_open=True,
            style={"position": "fixed", "top": 10, "right": 10}
        )
        return popup, data