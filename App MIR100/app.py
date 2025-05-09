# app.py

import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection
from utils.data import authenticate, user_credentials, update_password
import time
import rospy
from geometry_msgs.msg import Twist
from components.draw_mode import create_draw_mode_layout
import plotly.graph_objects as go
from function_draw_mode.draw_line_mode_callbacks import *
from function_teleop_control.teleop_control import TeleopControl
import random
from dash import callback_context
from components.rviz_section import create_rviz_section
import os
import yaml
from dash.exceptions import PreventUpdate

# Import callback cho Draw Line
from function_draw_control.draw_line_callbacks import register_draw_line_callbacks
from function_draw_control.draw_arc_callbacks import register_draw_arc_callbacks

app = dash.Dash(
    __name__,
    external_stylesheets=[
        dbc.themes.BOOTSTRAP,
        "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css",
    ],
    suppress_callback_exceptions=True,
    external_scripts=["assets/style.js"]
)

if not rospy.core.is_initialized():
    try:
        rospy.init_node('dash_app', anonymous=True)
        print("ROS node 'dash_app' initialized successfully.")
    except rospy.exceptions.ROSException as e:
        print(f"Error initializing ROS node: {e}")

if rospy.core.is_initialized():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
else:
    pub = None
    print("ROS not properly initialized, publisher not created.")

linear_speed = 0.5
angular_speed = 0.3

if pub:
    joystick_control = TeleopControl(pub, linear_speed, angular_speed)
else:
    class DummyTeleopControl:
        def __init__(self):
            pass
        def create_joystick_popup(self):
            return html.Div("ROS not available, TeleopControl is disabled.")
    joystick_control = DummyTeleopControl()

login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()

app.layout = html.Div(
    [
        dcc.Location(id='url', refresh=False),
        html.Div(id="app-container", children=[login_page.layout]),
        html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
        dcc.Interval(id='interval-component', interval=100, n_intervals=0)
    ]
)

@app.callback(
    Output("app-container", "children"),
    Input("login-button", "n_clicks"),
    State("username", "value"),
    State("password", "value"),
    prevent_initial_call=True
)
def login(n_clicks, username, password):
    if authenticate(username, password):
        return html.Div(
            [
                dcc.Location(id='url', refresh=False),
                status_bar.create_status_bar(),
                sidebar.create_sidebar(),
                html.Div(id="page-content", style={"marginLeft": "250px"}),
            ],
            style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
        )
    else:
        return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])

@app.callback(
    Output('page-content', 'children'),
    Input('url', 'pathname')
)
def display_page(pathname):
    if pathname == '/draw-mode':
        return create_draw_mode_layout()
    elif pathname == '/maps':
        return map_section.create_map_section()
    elif pathname == '/change-password':
        return change_password_page.layout
    elif pathname == '/rviz':
        return create_rviz_section()
    elif pathname == '/draw-control':
        map_info_path = "/home/duc/Downloads/App MIR100/static/map_image.info"
        try:
            with open(map_info_path, 'r') as file:
                map_info = yaml.safe_load(file)
                resolution = map_info['resolution']
                width = map_info['width']
                height = map_info['height']
                origin_x = map_info['origin_x']
                origin_y = map_info['origin_y']
        except:
            resolution, width, height = 0.05, 400, 400
            origin_x, origin_y = 0.0, 0.0

        real_width = width * resolution
        real_height = height * resolution

        # Cố định layout ngay khi tạo Figure
        fig = go.Figure(
            layout=dict(
                images=[dict(
                    source="/static/map_image.png",
                    xref="x",
                    yref="y",
                    x=origin_x,
                    y=origin_y + real_height,
                    sizex=real_width,
                    sizey=real_height,
                    sizing="stretch",
                    opacity=0.8,
                    layer="below")],
                xaxis=dict(
                    range=[origin_x, origin_x + real_width],
                    tick0=origin_x,
                    dtick=1.0,
                    gridcolor="rgba(255, 255, 255, 0.5)",
                    gridwidth=1,
                    showgrid=True,
                    scaleanchor="y",  # Removed to avoid infinite loop
                    scaleratio=1
                ),
                yaxis=dict(
                    range=[origin_y, origin_y + real_height],
                    tick0=origin_y,
                    dtick=1.0,
                    gridcolor="rgba(255, 255, 255, 0.5)",
                    gridwidth=1,
                    showgrid=True,
                    scaleanchor="x",  # Removed to avoid infinite loop
                    scaleratio=1
                ),
                plot_bgcolor="rgba(0,0,0,0)",
                paper_bgcolor="rgba(0,0,0,0)",
                autosize=True,
                margin=dict(l=0, r=0, t=0, b=0)
            )
        )
        #In thử xem layout có giá trị gì ko
        print(f"Initial fig.layout: {fig.layout}")

        return html.Div([
            dbc.Row([
                dbc.Col([
                    dbc.Button("Draw Line", id="draw-line-btn", color="primary", className="m-1"),
                    dbc.Button("Draw Arc", id="draw-arc-btn", color="primary", className="m-1"),
                    dbc.Button("Draw Polyline", id="draw-polyline-btn", color="primary", className="m-1"),
                    dbc.Button("Draw Spline 3", id="draw-spline-btn", color="primary", className="m-1"),
                    dbc.Button("Clear Line", id="clear-line-btn", color="danger", className="m-1"),
                    dbc.Button("Save Line", id="save-line-btn", color="success", className="m-1"),
                ], width=12)
            ], className="mb-3"),
            dcc.Graph(
                id="draw-control-graph",
                figure=fig,
                style={"height": "80vh", "width": "100%"},
                config={"displayModeBar": True, "scrollZoom": True, "responsive": True}
            ),
            dcc.Store(id="draw-data-store", data={"lines": [], "arcs": [], "polylines": [], "splines": [], "temp_points": []}),
            html.Div(id="draw-control-popups")  # Container cho popup
        ], style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
            "overflow": "hidden"
        })
    else:
        return html.Div(
            [
                status_bar.create_status_bar(),
                map_section.create_map_section(),
                html.Div(id="joystick-popup-container"),
                html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                dcc.Interval(id='interval-component', interval=1*1000, n_intervals=0)
            ]
        )

# Đăng ký callback cho Draw Line
# register_draw_line_callbacks(app)
register_draw_arc_callbacks(app)

# Các callback khác (giữ nguyên)
@app.callback(
    Output("password-status", "children"),
    Input("update-password-button", "n_clicks"),
    State("new-password", "value"),
    State("confirm-password", "value"),
    prevent_initial_call=True
)
def update_password_callback(n_clicks, new_password, confirm_password):
    if new_password == confirm_password:
        global user_credentials
        username = list(user_credentials.keys())[0]
        if update_password(username, new_password):
            return html.Div("Password updated successfully!", style={"color": "green"})
        else:
            return html.Div("Failed to update password.", style={"color": "red"})
    else:
        return html.Div("Passwords do not match.", style={"color": "red"})

@app.callback(
    Output("joystick-popup-container", "children"),
    Input("open-joystick-btn", "n_clicks"),
    prevent_initial_call=True,
)
def open_joystick(n_clicks):
    return joystick_control.create_joystick_popup()

@app.callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True
)
def close_joystick(n_clicks, is_open):
    return not is_open

@app.callback(
    [Output("linear-speed-display", "children"),
     Output("angular-speed-display", "children")],
    [Input("linear-speed-input", "value"),
     Input("angular-speed-input", "value")],
    prevent_initial_call=True
)
def update_speed(linear_speed_value, angular_speed_value):
    global joystick_control
    linear_speed = linear_speed_value
    angular_speed = angular_speed_value
    joystick_control.linear_speed = linear_speed
    joystick_control.angular_speed = angular_speed
    return (
        f"Linear Speed: {linear_speed_value}",
        f"Angular Speed: {angular_speed_value}",
    )

@app.callback(
    Output("joystick-output", "children"),
    [Input("forward-button", "n_clicks_timestamp"),
     Input("backward-button", "n_clicks_timestamp"),
     Input("left-button", "n_clicks_timestamp"),
     Input("right-button", "n_clicks_timestamp"),
     Input("forward-left-button", "n_clicks_timestamp"),
     Input("forward-right-button", "n_clicks_timestamp"),
     Input("back-left-button", "n_clicks_timestamp"),
     Input("back-right-button", "n_clicks_timestamp"),
     Input("stop-button", "n_clicks")],
    prevent_initial_call=True
)
def teleop_control(f, b, l, r, fl, fr, bl, br, s):
    ctx = callback_context
    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None
    if triggered_id == "forward-button":
        joystick_control.move_forward()
        return "Moving Forward"
    elif triggered_id == "backward-button":
        joystick_control.move_backward()
        return "Moving Backward"
    elif triggered_id == "left-button":
        joystick_control.turn_left()
        return "Turning Left"
    elif triggered_id == "right-button":
        joystick_control.turn_right()
        return "Turning Right"
    elif triggered_id == "forward-left-button":
        joystick_control.move_forward_left()
        return "Moving Forward Left"
    elif triggered_id == "forward-right-button":
        joystick_control.move_forward_right()
        return "Moving Forward Right"
    elif triggered_id == "back-left-button":
        joystick_control.move_backward_left()
        return "Moving Backward Left"
    elif triggered_id == "back-right-button":
        joystick_control.move_backward_right()
        return "Moving Backward Right"
    elif triggered_id == "stop-button":
        joystick_control.stop()
        return "Stop"
    else:
        return "No movement"

@app.callback(
    Output('map_section', 'children'),
    Input('language-dropdown', 'value')
)
def change_language(language):
    translations = {
        'en': {'title': 'Main Floor', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'vi': {'title': 'Tầng Chính', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'es': {'title': 'Piso Principal', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
    }
    translation = translations.get(language, translations['en'])
    return html.Div(
        [
            html.H3(translation['title'], className="mb-3", style={"color": "#2C3E50"}),
            html.P(translation['map'], className="text-muted"),
            html.Img(src="/path/to/save/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
            html.P(translation['ready'], className="text-info mt-2"),
            html.Div(id="content-area"),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )

@app.callback(
    Output("map-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image(n):
    timestamp = int(time.time())
    return f"/static/map_image.png?{timestamp}"

@app.callback(
    Output("lidar-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_lidar_image(n):
    timestamp = int(time.time())
    return f"/static/lidar_image.png?{timestamp}"

@app.callback(
    [Output("lidar-f-image", "src"), Output("lidar-b-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_lidar_images(n):
    timestamp = int(time.time())
    return (
        f"/static/f_scan_image.png?{timestamp}",
        f"/static/b_scan_image.png?{timestamp}"
    )

@app.callback(
    [Output("path-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_path_image(n):
    random_value = random.randint(1, 100000)
    return (
        f"/static/path_image.png?random={random_value}",
    )

last_modified_time = 0

@app.callback(
    [Output("robot-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_robot_image(n):
    global last_modified_time
    image_path = "/home/duc/Downloads/App MIR100/static/robot_image.png"
    try:
        modified_time = os.path.getmtime(image_path)
    except FileNotFoundError:
        print(f"Error: Image file not found: {image_path}")
        return [dash.no_update]
    except Exception as e:
        # print(f"Other error gettingf"Other error getting modified time: {e}")
        return [dash.no_update]
    if modified_time > last_modified_time:
        last_modified_time = modified_time
        return [f"/static/robot_image.png?time={modified_time}"]
    else:
        raise PreventUpdate

@app.callback(
    Output("map-image-draw-mode", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image_draw_mode(n):
    timestamp = int(time.time())
    return f"/static/map_image.png?{timestamp}"

@app.callback(
    Output("sidebar-nav", "children"),
    Input('url', 'pathname'),
    prevent_initial_call=True
)
def update_active_link(pathname):
    nav_links = [
        {"href": "/", "id": "index-link", "label": "Home"},
        {"href": "/draw-mode", "id": "draw-mode-link", "label": "Draw Mode"},
        {"href": "/rviz", "id": "rviz-link", "label": "RViz"},
        {"href": "/draw-control", "id": "draw-control-link", "label": "Draw & Control"},
        {"href": "#", "id": "sounds-link", "label": "Sounds"},
        {"href": "#", "id": "transitions-link", "label": "Transitions"},
        {"href": "#", "id": "io-modules-link", "label": "I/O Modules"},
        {"href": "#", "id": "users-link", "label": "Users"},
        {"href": "#", "id": "user-groups-link", "label": "User Groups"},
        {"href": "#", "id": "paths-link", "label": "Paths"},
        {"href": "#", "id": "path-guides-link", "label": "Path Guides"},
        {"href": "#", "id": "marker-types-link", "label": "Marker Types"},
        {"href": "#", "id": "footprints-link", "label": "Footprints"},
        {"href": "/change-password", "id": "change-password-link", "label": "Change Password"},
    ]

    updated_links = []
    for link in nav_links:
        active = pathname == link["href"]
        updated_links.append(
            dbc.NavLink(
                link["label"],
                href=link["href"],
                id=link["id"],
                className="text-white",
                active=active
            )
        )
    return updated_links

if __name__ == "__main__":
    app.run_server(debug=True)