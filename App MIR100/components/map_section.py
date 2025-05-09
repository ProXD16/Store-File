from dash import dcc, html
import dash_bootstrap_components as dbc

class MapSection:
    def create_map_section(self):
        return html.Div(
            [
                html.H3("HOME", className="mb-3", style={"color": "#2C3E50"}),
                html.P("Edit and draw the map", className="text-muted"),
                # Container for map and lidar images
                html.Div(
                    [
                        # Map image
                        html.Img(
                            id="map-image",
                            src="/static/map_image.png",
                            style={
                                "width": "800px",  # Fixed size
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",  # Ensure the image is not distorted
                                "position": "absolute",  # Position the map image
                                "z-index": "1",  # Map image is below lidar image
                            },
                        ),
                        # Lidar image (front scan)
                        html.Img(
                            id="lidar-f-image",
                            src="/static/f_scan_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "2",  # Lidar image is above map image
                            },
                        ),
                        # Lidar image (back scan)
                        html.Img(
                            id="lidar-b-image",
                            src="/static/b_scan_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "2",  # Lidar image is above map image
                            },
                        ),
                        html.Img(
                            id="path-image",
                            src="/static/path_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "3",  # Lidar image is above map image
                            },
                        ),
                        html.Img(
                            id="robot-image",
                            src="/static/robot_image.png",
                            style={
                                "width": "800px",  # Same size as map image
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  # Position the lidar image
                                "top": "0",  # Align with map image
                                "left": "0",  # Align with map image
                                "z-index": "4",  # Lidar image is above map image
                            },
                        ),
                    ],
                    style={
                        "position": "relative",  # Container for absolute positioning
                        "width": "800px",  # Same size as images
                        "height": "600px",
                    },
                ),
                html.P("The map is ready for your work.", className="text-info mt-2"),
                html.Div(id="content-area"),  # Placeholder for content based on sidebar selection
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,  # Update every second
                    n_intervals=0
                )
            ],
            style={
                "padding": "20px",
                "flex": "1",
                "background": "#ECF0F1",
                "marginLeft": "250px",
                "marginTop": "50px",
            },
        )