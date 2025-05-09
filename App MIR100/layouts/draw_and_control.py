# layouts/draw_and_control.py
from dash import html, dcc
import dash_bootstrap_components as dbc

def create_draw_and_control_layout():
    return html.Div([
        dbc.Row([
            dbc.Col(md=3, children=[
                html.H4("Drawing Tools"),
                dbc.Button("Draw Line", id="draw-line-button", color="primary", className="mb-2"),
                dbc.Button("Draw Arc", id="draw-arc-button", color="primary", className="mb-2"),
                dbc.Button("Draw Polyline", id="draw-polyline-button", color="primary", className="mb-2"),
                dbc.Button("Draw Spline (Bậc 3)", id="draw-spline-button", color="primary", className="mb-2"),
                dbc.Button("Clear Line", id="clear-line-button", color="danger", className="mb-2"),
                dbc.Button("Save Line", id="save-line-button", color="success"),
            ]),
            dbc.Col(md=9, children=[
                html.H4("Map"),
                dcc.Graph(
                    id='map-graph',
                    figure={
                        'data': [],
                        'layout': {
                            'xaxis': {'range': [-10, 10], 'title': 'X (m)'},  # Adjust ranges as needed
                            'yaxis': {'range': [-10, 10], 'title': 'Y (m)'},  # Adjust ranges as needed
                            'images': [{
                                'source': "/static/map_image.png",
                                'xref': "x",
                                'yref': "y",
                                'x': -10,  # Adjust based on your map's bounds
                                'y': 10,  # Adjust based on your map's bounds
                                'sizex': 20,  # Adjust based on your map's bounds
                                'sizey': 20,  # Adjust based on your map's bounds
                                'sizing': "stretch",
                                'layer': "below"
                            }],
                            'grid': {'rows': 20, 'columns': 20, 'pattern': 'independent'}, #Adjust based on range
                            'margin': {'l': 0, 'r': 0, 't': 0, 'b': 0}
                        }
                    },
                    style={'height': '800px'} #Adjust as needed
                ),
            ])
        ])
    ])