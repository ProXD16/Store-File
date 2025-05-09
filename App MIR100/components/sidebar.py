from dash import dcc, html
import dash_bootstrap_components as dbc

class Sidebar:
    def create_sidebar(self):
        return html.Div(
            [
                dcc.Link(
                    html.H2("GUI MIR100", className="text-white text-center"),
                    href="/",
                    style={"text-decoration": "none", "color": "white"},
                ),
                html.Hr(className="text-white"),
                dbc.Nav(
                    [
                        dbc.NavLink("Draw Mode", href="/draw-mode", id="draw-mode-link", className="text-white"),
                        dbc.NavLink("RViz", href="/rviz", id="rviz-link", className="text-white"),
                        dbc.NavLink("Draw & Control", href="/draw-control", id="draw-control-link", className="text-white"),  # Thêm mục mới
                        dbc.NavLink("Sounds", href="#", id="sounds-link", className="text-white"),
                        dbc.NavLink("Transitions", href="#", id="transitions-link", className="text-white"),
                        dbc.NavLink("I/O Modules", href="#", id="io-modules-link", className="text-white"),
                        dbc.NavLink("Users", href="#", id="users-link", className="text-white"),
                        dbc.NavLink("User Groups", href="#", id="user-groups-link", className="text-white"),
                        dbc.NavLink("Paths", href="#", id="paths-link", className="text-white"),
                        dbc.NavLink("Path Guides", href="#", id="path-guides-link", className="text-white"),
                        dbc.NavLink("Marker Types", href="#", id="marker-types-link", className="text-white"),
                        dbc.NavLink("Footprints", href="#", id="footprints-link", className="text-white"),
                        dbc.NavLink("Change Password", href="/change-password", id="change-password-link", className="text-white"),
                    ],
                    vertical=True,
                    pills=True,
                    id="sidebar-nav",
                ),
            ],
            style={
                "background-color": "#2C3E50",
                "padding": "20px",
                "width": "250px",
                "height": "100vh",
                "color": "white",
                "position": "fixed",
            },
        )