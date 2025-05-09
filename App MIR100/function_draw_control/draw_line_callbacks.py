import dash
from dash import dcc, html, Input, Output, State, ALL
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import yaml
from dash.exceptions import PreventUpdate
import json

def register_draw_line_callbacks(app):
    @app.callback(
        Output("draw-control-popups", "children"),
        Input("draw-line-btn", "n_clicks"),
        State("draw-control-popups", "children"),
        prevent_initial_call=True
    )
    def show_draw_line_popup(n_clicks, existing_popups):
        if n_clicks is None:
            return existing_popups  # No click, return existing popups (if any)

        popup = dbc.Modal(
            [
                dbc.ModalHeader(dbc.ModalTitle("Draw Line Mode")),
                dbc.ModalBody(
                    [
                        dbc.RadioItems(
                            options=[
                                {"label": "Manual (Click & Drag)", "value": "manual"},
                                {"label": "Enter Coordinates", "value": "coordinates"},
                            ],
                            value="manual",
                            id="draw-line-mode-radio",
                            className="mb-3",
                        ),
                        html.Div(id="draw-line-mode-content"),  # Content based on the selected mode
                    ]
                ),
                dbc.ModalFooter(
                    [
                        dbc.Button("Close", id="close-draw-line-modal", className="ms-auto"),
                    ]
                ),
            ],
            id="draw-line-modal",
            is_open=True,
        )

        if existing_popups is None:
            return popup
        else:
            return existing_popups + popup

    @app.callback(
        Output("draw-line-mode-content", "children"),
        Input("draw-line-mode-radio", "value")
    )
    def update_draw_line_content(mode):
        if mode == "manual":
            return html.Div([
                html.P("Click and drag to draw a line on the map."),
                dbc.Button("OK", id="draw-line-manual-ok", n_clicks=0, color="primary", className="mt-2") # Nút OK để kích hoạt chế độ vẽ
            ])
        elif mode == "coordinates":
            return html.Div(
                [
                    dbc.Row(
                        [
                            dbc.Col([dbc.Label("Start X"), dcc.Input(id="start-x", type="number", placeholder="X")]),
                            dbc.Col([dbc.Label("Start Y"), dcc.Input(id="start-y", type="number", placeholder="Y")]),
                        ]
                    ),
                    dbc.Row(
                        [
                            dbc.Col([dbc.Label("End X"), dcc.Input(id="end-x", type="number", placeholder="X")]),
                            dbc.Col([dbc.Label("End Y"), dcc.Input(id="end-y", type="number", placeholder="Y")]),
                        ]
                    ),
                    dbc.Button("Draw Line", id="draw-line-coords-button", color="primary", className="mt-2"),
                    html.Div(id="draw-line-coords-message", className="mt-2")
                ]
            )
        else:
            return ""

    @app.callback(
        [Output("draw-line-modal", "is_open"), Output("draw-control-popups", "children", allow_duplicate=True)],
        Input("close-draw-line-modal", "n_clicks"),
        State("draw-line-modal", "is_open"),
        prevent_initial_call=True
    )
    def close_draw_line_modal(n_clicks, is_open):
        if n_clicks:
            return False, None
        else:
            return is_open, dash.no_update

    @app.callback(
        Output("draw-control-graph", "figure", allow_duplicate=True),
        Output("draw-data-store", "data", allow_duplicate=True),
        Output("draw-line-coords-message", "children"),
        Input("draw-line-coords-button", "n_clicks"),
        State("start-x", "value"),
        State("start-y", "value"),
        State("end-x", "value"),
        State("end-y", "value"),
        State("draw-control-graph", "figure"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def draw_line_with_coordinates(n_clicks, start_x, start_y, end_x, end_y, figure, data):
        if n_clicks is None:
            raise PreventUpdate

        if not all([isinstance(val, (int, float)) for val in [start_x, start_y, end_x, end_y]]):
            return dash.no_update, dash.no_update, "Please enter valid numerical coordinates."

        new_line = {
            "x0": start_x,
            "y0": start_y,
            "x1": end_x,
            "y1": end_y
        }

        data["lines"].append(new_line)

        fig = go.Figure(figure)
        fig.add_shape(
            type="line",
            x0=start_x,
            y0=start_y,
            x1=end_x,
            y1=end_y,
            line=dict(color="blue", width=3)
        )
        return fig, data, f"Line drawn from ({start_x}, {start_y}) to ({end_x}, {end_y})"

    # Callback kích hoạt chế độ vẽ Manual VÀ đóng popup
    @app.callback(
            [Output("draw-control-graph", "config", allow_duplicate=True),
            Output("draw-line-modal", "is_open", allow_duplicate=True),
            Output("draw-control-popups", "children", allow_duplicate=True),
            Output("draw-control-graph", "figure", allow_duplicate=True)], #Thay layout thanh figure de lay figure.layout
            Input("draw-line-manual-ok", "n_clicks"),
            State("draw-line-mode-radio", "value"),
            State("draw-line-modal", "is_open"),
            State("draw-control-graph", "figure"), #Lay figure thay cho layout
            prevent_initial_call=True
        )
    def enable_manual_draw_and_close(n_clicks, draw_mode, is_open, figure): #Doi ten current_layout thanh figure
        print(f"enable_manual_draw_and_close called.  n_clicks: {n_clicks}, draw_mode: {draw_mode}, is_open: {is_open}, figure: {figure}") # Debug
        if draw_mode == "manual" and n_clicks > 0:
            # Kiểm tra nếu layout là None, tạo một layout mới
            if figure is None or figure.get('layout') is None:
                print("figure is None or figure.layout is None, creating a new layout.") # Debug
                new_layout = {'dragmode': 'drawline'}  # Khởi tạo layout nếu nó là None
            else:
                print("figure is NOT None and figure.layout is NOT None, copying the existing layout.") # Debug
                new_layout = figure['layout'].copy() # Sao chép layout từ figure
            new_layout['dragmode'] = 'drawline'  # Cập nhật dragmode trong layout
            print(f"New layout: {new_layout}") # Debug
            new_figure = go.Figure(figure) #Tao mot figure moi tu figure cu
            new_figure.update_layout(new_layout) #Update layout cho figure moi
            return {"modeBarButtonsToAdd": ['drawline']}, False, None, new_figure #Tra ve figure moi
        else:
            print("draw_mode is not manual or n_clicks is not greater than 0, not enabling draw mode.") # Debug
            return {"modeBarButtonsToAdd": []}, is_open, dash.no_update, dash.no_update #Khong tra ve layout nua

    # Callback xử lý sự kiện vẽ đường thủ công
    @app.callback(
        Output("draw-data-store", "data", allow_duplicate=True),
        Input("draw-control-graph", "relayoutData"),
        State("draw-data-store", "data"),
        prevent_initial_call=True
    )
    def handle_manual_draw(relayoutData, data):
        if relayoutData and 'shapes' in relayoutData:
            new_shape = relayoutData['shapes'][-1]  # Lấy shape vừa vẽ
            if new_shape['type'] == 'line':
                data["lines"].append({
                    "x0": new_shape['x0'],
                    "y0": new_shape['y0'],
                    "x1": new_shape['x1'],
                    "y1": new_shape['y1']
                })
        return data

    @app.callback(
        Output("draw-control-graph", "figure", allow_duplicate=True),
        Input("draw-data-store", "data"),
        State("draw-control-graph", "figure"),
        prevent_initial_call=True
    )
    def redraw_lines(data, figure):
        fig = go.Figure(figure)
        for line in data["lines"]:
            fig.add_shape(
                type="line",
                x0=line['x0'],
                y0=line['y0'],
                x1=line['x1'],
                y1=line['y1'],
                line=dict(color="blue", width=3)
            )
        return fig