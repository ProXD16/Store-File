from dash import dcc, html
import dash_bootstrap_components as dbc
from utils.data import LANGUAGES

class StatusBar:
    def create_status_bar(self):
        return html.Div(
            [
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                html.Span("No missions in queue", className="badge bg-warning text-dark me-2"),
                                html.Span("PAUSED", className="badge bg-danger me-2"),
                                html.Span("ALL OK", className="badge bg-success me-2"),
                                html.Span(id="linear-speed-display", className="badge bg-info text-dark me-2", children="Linear Speed: 0.5"),
                                html.Span(id="angular-speed-display", className="badge bg-info text-dark me-2", children="Angular Speed: 0.3"),
                            ],
                            width="auto",
                        ),
                        dbc.Col(
                            dcc.Dropdown(
                                id="language-dropdown",
                                options=LANGUAGES,
                                value="en",  # Default language
                                clearable=False,
                            ),
                            width="auto",
                            className="me-2"
                        ),
                         dbc.Col(
                            [
                                html.Div("Linear Speed:", style={"marginRight": "5px", "color": "white"}), # Add label for input long speed.
                                dbc.Input(id="linear-speed-input", type="number", placeholder="Linear Speed", value=0.5, step=0.1, style={"width": "100px", "marginRight": "10px"}),
                                html.Div("Angular Speed:", style={"marginLeft": "10px","marginRight": "5px", "color": "white"}),#Add label for input angular speed.
                                dbc.Input(id="angular-speed-input", type="number", placeholder="Angular Speed", value=0.3, step=0.1, style={"width": "100px"}),
                            ],
                            width="auto",
                            className="me-2",
                            style={"display": "flex", "alignItems": "center"}
                        ),
                        dbc.Col(
                            dbc.Button("Open Teleoperation", id="open-joystick-btn", color="primary", size="sm"),
                            width="auto",
                            className="text-end",
                        ),

                    ],
                    align="center",
                    className="g-0",
                    style={"width": "100%"},
                )
            ],
            style={
                "background": "#34495E",
                "color": "white",
                "padding": "10px",
                "position": "fixed",
                "top": 0,
                "left": "250px",
                "width": "100%",
                "zIndex": 1000,
            },
        )