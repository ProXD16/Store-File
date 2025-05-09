# rviz_section.py
from dash import html, dcc, callback, Input, Output, State, no_update
import dash_bootstrap_components as dbc
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from PIL import Image
import plotly.express as px
import dash
import io
import base64
import json
import os

class RVizSection:
    FIXED_ARROW_LENGTH = 30

    def __init__(self, goal_topic="/move_base_simple/goal"):
        self.goal_topic = goal_topic
        try:
            self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        except rospy.exceptions.ROSException as e:
            print(f"Error connecting to ROS: {e}")
            self.goal_pub = None
        self.map_data = self.load_image_as_numpy("/home/duc/Downloads/App MIR100/static/map_image.png")

    def load_image_as_numpy(self, image_path):
        try:
            img = Image.open(image_path).convert('L')
            map_data = np.array(img)
            if len(map_data.shape) > 2:
                print("WARNING: Image has more than two dimensions, potential color issue")
            return map_data
        except FileNotFoundError:
            print(f"Error: Image file not found at {image_path}")
            return self.generate_fake_map()

    def generate_fake_map(self):
        size = 500
        return np.random.randint(0, 255, size=(size, size))

    def create_composite_image(self):
        try:
            map_img = Image.open("/home/duc/Downloads/App MIR100/static/map_image.png")
            if map_img.mode != 'RGBA':
                map_img = map_img.convert('RGBA')

            lidar_f_img = Image.open("/home/duc/Downloads/App MIR100/static/f_scan_image.png").convert("RGBA")
            lidar_b_img = Image.open("/home/duc/Downloads/App MIR100/static/b_scan_image.png").convert("RGBA")
            robot_img = Image.open("/home/duc/Downloads/App MIR100/static/robot_image.png").convert("RGBA")
            path_img = Image.open("/home/duc/Downloads/App MIR100/static/path_image.png").convert("RGBA")
        except FileNotFoundError as e:
            print(f"Error loading images: {e}")
            return None
        width, height = map_img.size
        lidar_f_img = lidar_f_img.resize((width, height), Image.LANCZOS)
        lidar_b_img = lidar_b_img.resize((width, height), Image.LANCZOS)
        robot_img = robot_img.resize((width, height), Image.LANCZOS)
        path_img = path_img.resize((width, height), Image.LANCZOS)
        lidar_blend_f = Image.blend(lidar_f_img, lidar_f_img, alpha=0.5)
        lidar_blend_b = Image.blend(lidar_blend_f, lidar_b_img, alpha=0.5)
        composed = Image.alpha_composite(lidar_blend_b, robot_img)
        composite_img = Image.alpha_composite(composed, path_img)
        white_background = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        final_img = Image.alpha_composite(white_background, composite_img)

        return final_img

    def create_figure(self, drawing_enabled=False, start_x=None, start_y=None, end_x=None, end_y=None):
        composite_img = self.create_composite_image()
        if composite_img is None:
            fake_map = self.generate_fake_map()
            fig = px.imshow(fake_map, color_continuous_scale='gray')
        else:
            img_byte_arr = io.BytesIO()
            composite_img.save(img_byte_arr, format='PNG')
            img_byte_arr = img_byte_arr.getvalue()
            encoded_image = base64.b64encode(img_byte_arr).decode()
            img_data = f'data:image/png;base64,{encoded_image}'
            fig = px.imshow(Image.open(io.BytesIO(base64.b64decode(encoded_image))),binary_string=True)
            fig.update_layout(

                images=[dict(
                source=img_data,

                xref="x", yref="y",
                x=0, y=composite_img.size[1],
                sizex=composite_img.size[0], sizey=composite_img.size[1],

                sizing="stretch",

                opacity=1,

                layer="below")],
                plot_bgcolor='rgba(255,255,255,0)',
                paper_bgcolor='rgba(255,255,255,0)'
            )


        dragmode = "drawline" if drawing_enabled else False
        fig.update_layout(
                dragmode=dragmode,
                xaxis=dict(showgrid=False, zeroline=False, visible=False),

                yaxis=dict(showgrid=False, zeroline=False, visible=False,scaleratio=1),
                margin=dict(l=0, r=0, b=0, t=0),

                height=600,
                width=800,
                newshape_line_color='red',


            coloraxis_showscale=False
            )
        fig.update_traces(
            hovertemplate=None,
            hoverinfo='skip'
        )

        if start_x is not None and start_y is not None and end_x is not None and end_y is not None:
            dx = end_x - start_x
            dy = end_y - start_y
            angle = math.atan2(dy, dx)
            fixed_end_x = start_x + self.FIXED_ARROW_LENGTH * math.cos(angle)
            fixed_end_y = start_y + self.FIXED_ARROW_LENGTH * math.sin(angle)
            fig.add_annotation(
                x=fixed_end_x, y=fixed_end_y, ax=start_x, ay=start_y,
                xref="x", yref="y", axref="x", ayref="y",
                arrowhead=2, arrowcolor='red', arrowwidth=2,
            )


        return fig

    def publish_goal(self, x, y, angle):
        if self.goal_pub:
            try:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.z = math.sin(angle / 2)
                pose.pose.orientation.w = math.cos(angle / 2)
                self.goal_pub.publish(pose)
                return "Goal published successfully! (x: {}, y: {}, angle: {})".format(x, y, angle)
            except Exception as e:
                return f"Error publishing goal: {e}"
        return "Goal publisher not initialized."

def create_rviz_section():
    layout = html.Div(
        [
            html.H3("RViz Interface", className="mb-3", style={"color": "#2C3E50"}),
            dbc.Row(
                [
                    dbc.Col(
                        html.Div(
                            [
                                html.Button("Send Goal", id="send-goal-btn", className="btn btn-primary me-2"),
                                html.Button("2D Nav Goal", id="nav-goal-btn", className="btn btn-secondary"),
                            ],
                            className="mb-3"
                        ),
                        width=12,
                    ),
                ]
            ),

            html.Div(
                [
                    dcc.Graph(
                        id="map-graph",
                        figure=RVizSection().create_figure(),
                        style={
                            "width": "800px",
                            "height": "600px",
                            "background": "rgba(0, 0, 0, 0)",
                            "position": "absolute",
                            "z-index": "2",
                            "top": "0",
                            "left": "0",
                        },
                        config={'scrollZoom': False, 'displayModeBar': False}
                    ),
                     html.Img(
                        id="map-image",
                        src="/static/map_image.png",
                        style={
                            "width": "800px",
                            "height": "600px",
                            "border": "2px solid #34495E",
                            "object-fit": "contain",
                            "position": "absolute",
                            "z-index": "1",
                            "top": "0",
                            "left": "0",
                        },
                    ),
                ],
                style={"position": "relative", "width": "800px", "height": "600px"},
            ),


            html.P("Draw line and after release, it sends the goal", className="text-info mt-2"),
            html.Div(id="goal-status"),
            dcc.Interval(
                id='interval-component',
                interval=1 * 1000,
                n_intervals=0
            ),
            dcc.Store(id="drag-start-coords", data=None),
            dcc.Store(id="drawing-enabled", data=False),
            dcc.Store(id="latest-goal", data=None),
            dbc.Modal(
                [
                    dbc.ModalHeader(dbc.ModalTitle("Enter Goal Coordinates")),
                    dbc.ModalBody(
                        dbc.Form(
                            [
                                dbc.Row(
                                    [
                                        dbc.Col(dbc.Label("X:", html_for="goal-x")),
                                        dbc.Col(dbc.Input(type="number", id="goal-x", placeholder="X Coordinate")),
                                    ],
                                    className="mb-3",
                                ),
                                dbc.Row(
                                    [
                                        dbc.Col(dbc.Label("Y:", html_for="goal-y")),
                                        dbc.Col(dbc.Input(type="number", id="goal-y", placeholder="Y Coordinate")),
                                    ],
                                    className="mb-3",
                                ),
                                dbc.Row(
                                    [
                                        dbc.Col(dbc.Label("Z:", html_for="goal-z")),
                                        dbc.Col(dbc.Input(type="number", id="goal-z", placeholder="Z Coordinate", value=0)),
                                    ],
                                    className="mb-3",
                                ),
                                dbc.Row(
                                    [
                                        dbc.Col(dbc.Label("W:", html_for="goal-w")),
                                        dbc.Col(dbc.Input(type="number", id="goal-w", placeholder="W Orientation", value=1)),
                                    ],
                                    className="mb-3",
                                ),
                            ]
                        )
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button("Close", id="close-goal-modal", className="ms-auto"),
                            dbc.Button("Send", id="send-goal-modal-btn", color="primary", className="ms-2"),
                        ]
                    ),
                ],
                id="goal-modal",
                is_open=False,
            ),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )
    return layout

def load_map_info():
    map_info_path = "/home/duc/Downloads/App MIR100/static/map_image.json"
    if os.path.exists(map_info_path):
        with open(map_info_path, "r") as f:
            return json.load(f)
    else:
        print("WARNING: map_info.json not found. Using default values or disabling auto-goal sending.")
        return None # Modified return

# Callbacks
@callback(
    Output("goal-modal", "is_open"),
    [Input("send-goal-btn", "n_clicks"), Input("close-goal-modal", "n_clicks"), Input("send-goal-modal-btn", "n_clicks")],
    [State("goal-modal", "is_open")],
    prevent_initial_call=True,
)
def toggle_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "send-goal-btn":
        return True
    elif button_id == "close-goal-modal" or button_id == "send-goal-modal-btn":
        return False
    return is_open

@callback(
    Output("goal-status", "children"),
    [Input("send-goal-modal-btn", "n_clicks")],
    [
        State("goal-x", "value"),
        State("goal-y", "value"),
        State("goal-z", "value"),
        State("goal-w", "value"),
    ],
    prevent_initial_call=True,
)
def send_goal_coordinates(n_clicks, x, y, z, w):
    if n_clicks:
        rviz_section = RVizSection()
        angle = math.atan2(float(y), float(x))
        status = rviz_section.publish_goal(x, y, angle)
        return status
    return no_update

@callback(
    Output("map-graph", "figure"),
    [Input("map-graph", "clickData"), Input("map-graph", "relayoutData"), Input("interval-component", "n_intervals")],
    [State("map-graph", "figure"), State("drag-start-coords", "data"), State("drawing-enabled", "data")],
    prevent_initial_call=True,
)
def update_map(clickData, relayoutData, n_intervals, existing_map, drag_start_coords, drawing_enabled):
    ctx = dash.callback_context
    rviz_section = RVizSection()
    if not ctx.triggered:
        return existing_map
    trigger_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if trigger_id == "map-graph" and clickData and drawing_enabled:
        start_x = clickData['points'][0]['x']
        start_y = clickData['points'][0]['y']
        return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=start_x, end_y=start_y)

    elif trigger_id == "map-graph" and relayoutData and drag_start_coords and drawing_enabled:
        start_x = drag_start_coords["start_x"]
        start_y = drag_start_coords["start_y"]
        end_x = relayoutData.get("xaxis.range[0]")
        end_y = relayoutData.get("yaxis.range[1]")
        return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=end_x, end_y=end_y)

    elif trigger_id == "interval-component":
        return rviz_section.create_figure(drawing_enabled)

    return existing_map

@callback(
    Output("drag-start-coords", "data"),
    [Input("map-graph", "clickData")],
    [State("drawing-enabled", "data")],
    prevent_initial_call=True
)
def store_drag_start_coords(clickData, drawing_enabled):
    if clickData and drawing_enabled:
        return {"start_x": clickData['points'][0]['x'], "start_y": clickData['points'][0]['y']}
    return no_update

@callback(
    [Output("drawing-enabled", "data"), Output("nav-goal-btn", "className")],
    [Input("nav-goal-btn", "n_clicks")],
    [State("drawing-enabled", "data")],
    prevent_initial_call=True
)
def toggle_drawing_mode(n_clicks, drawing_enabled):
    if n_clicks:
        drawing_enabled = not drawing_enabled
        button_class = "btn btn-success" if drawing_enabled else "btn btn-secondary"
        return drawing_enabled, button_class
    return drawing_enabled, "btn btn-secondary"


@callback(
    [Output("goal-status", "children", allow_duplicate=True),
     Output("drawing-enabled", "data", allow_duplicate=True),
     Output("nav-goal-btn", "className", allow_duplicate=True)],
    Input("map-graph", "relayoutData"),
    [State("drag-start-coords", "data"), State("drawing-enabled", "data")],
    prevent_initial_call=True
)
def auto_send_goal(relayoutData, drag_start_coords, drawing_enabled):
    if not (relayoutData and drawing_enabled):
        print("Không thỏa mãn điều kiện gửi goal")
        return no_update, no_update, no_update

    map_info = load_map_info()

    if not map_info:
        print("Không thể tải thông tin bản đồ từ map_info.json. Không tự động gửi goal.")
        return "Không thể gửi goal tự động do thiếu thông tin bản đồ.", no_update, no_update

    if "shapes" not in relayoutData or not relayoutData["shapes"]:
        print("relayoutData không có shapes hợp lệ!")
        return no_update, no_update, no_update

    shape = relayoutData["shapes"][0]
    x_pixel_start, y_pixel_start = shape["x0"], shape["y0"]
    x_pixel_end, y_pixel_end = shape["x1"], shape["y1"]

    start_x = map_info["origin_x"] + (x_pixel_start * map_info["resolution"])
    start_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_start) * map_info["resolution"])
    end_x = map_info["origin_x"] + (x_pixel_end * map_info["resolution"])
    end_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_end) * map_info["resolution"])

    dx = end_x - start_x
    dy = end_y - start_y
    angle = math.atan2(dy, dx)

    # Gửi tọa độ đến robot qua ROS
    rviz_section = RVizSection()
    status = rviz_section.publish_goal(start_x, start_y, angle)

    # Tắt drawing mode sau khi gửi goal
    drawing_enabled = False
    button_class = "btn btn-secondary"
    return status, drawing_enabled, button_class