# draw_mode_callbacks.py
from dash import Input, Output, State, callback, callback_context, no_update
import plotly.graph_objects as go
import json

# Callback to open the draw method modal
@callback(
    Output("draw-method-modal", "is_open"),
    Input("draw-line-button", "n_clicks"),
    State("draw-method-modal", "is_open"),
    prevent_initial_call=True,
)
def open_draw_method_modal(n_clicks, is_open):
    return not is_open

# Callback to set the draw method (manual or coordinate) and close the modal
@callback(
    Output("draw-method", "data"),
    Output("draw-method-modal", "is_open", allow_duplicate=True),  # Add output to close the modal
    Input("manual-draw-button", "n_clicks"),
    Input("coordinate-draw-button", "n_clicks"),
    State("draw-method-modal", "is_open"),
    prevent_initial_call=True,
)
def set_draw_method(manual_clicks, coordinate_clicks, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update, no_update
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-button":
        return "manual", False  # Close the modal when manual is chosen
    elif button_id == "coordinate-draw-button":
        return "coordinate", False  # Close the modal when coordinate is chosen
    return "", is_open # Return is_open to avoid change modal status.

# Callback to open coordinate modal when "coordinate" draw method is selected
@callback(
    Output("coordinate-modal", "is_open"),
    Input("draw-method", "data"),
    State("coordinate-modal", "is_open"),
    prevent_initial_call=True,
)
def open_coordinate_modal(draw_method, is_open):
    if draw_method == "coordinate":
        return True
    return False

# Callback to draw line based on coordinates entered in the modal
@callback(
    Output("map-image-draw-mode", "figure"),
    Input("draw-button", "n_clicks"),
    State("start-x", "value"),
    State("start-y", "value"),
    State("end-x", "value"),
    State("end-y", "value"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_line_coordinate(n_clicks, start_x, start_y, end_x, end_y, figure):
    if n_clicks is None:
        return no_update

    try:
        start_x = float(start_x)
        start_y = float(start_y)
        end_x = float(end_x)
        end_y = float(end_y)
    except (ValueError, TypeError):
        print("Invalid coordinates entered.")  # Log the error
        return figure  # Return the figure unchanged

    # Add line to the figure
    figure["data"].append(
        go.Scatter(
            x=[start_x, end_x],
            y=[start_y, end_y],
            mode="lines",
            line=dict(color="blue", width=2),
            showlegend=False,
        )
    )
    return figure

# Callback to store the start point when the user clicks on the graph
@callback(
    Output("line-coordinates", "data"),
    Input("map-image-draw-mode", "clickData"),
    State("draw-method", "data"),
    prevent_initial_call=True,
)
def store_start_point(clickData, draw_method):
    print("store_start_point called")  # Add this line
    print("draw_method:", draw_method)  # Add this line
    print("clickData:", clickData) # Add this line

    if draw_method == "manual" and clickData:
        x = clickData["points"][0]["x"]
        y = clickData["points"][0]["y"]
        return {"start_x": x, "start_y": y}
    return {}

# Callback to draw the line when the user releases the mouse (relayoutData)
@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("map-image-draw-mode", "relayoutData"),
    Input("draw-method", "data"),  # Add draw-method as an Input
    State("line-coordinates", "data"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_line_on_release(relayoutData,draw_method, line_coordinates, figure):
    print("draw_line_on_release called")
    print("draw_method:", draw_method)
    print("relayoutData:", json.dumps(relayoutData))  # Print relayoutData
    print("line_coordinates:", line_coordinates) # Add this line

    if draw_method == "manual" and relayoutData and line_coordinates and "start_x" in line_coordinates:
        # Check if relayoutData is due to a drag/release (mouse up)
        if 'xaxis.range[0]' in relayoutData and 'yaxis.range[0]' in relayoutData and 'xaxis.range[1]' in relayoutData and 'yaxis.range[1]' in relayoutData:
            # Extract start point from stored coordinates
            start_x = line_coordinates["start_x"]
            start_y = line_coordinates["start_y"]

            # Extract end point from relayoutData (mouse release)
            end_x = relayoutData['xaxis.range[1]']
            end_y = relayoutData['yaxis.range[0]']

            # Add line to the figure
            figure["data"].append(
                go.Scatter(
                    x=[start_x, end_x],
                    y=[start_y, end_y],
                    mode="lines",
                    line=dict(color="red", width=2),
                    showlegend=False,
                )
            )
            return figure
        else:
            return no_update
    else:
        return no_update

# Callback to clear the stored start point after drawing
@callback(
    Output("line-coordinates", "data", allow_duplicate=True),
    Input("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def clear_start_point(figure):
    return {}


# # draw_mode_callbacks.py
# from dash import Input, Output, State, callback, callback_context, no_update
# import plotly.graph_objects as go
# import json

# # Callback to open the draw method modal
# @callback(
#     Output("draw-method-modal", "is_open"),
#     Input("draw-line-button", "n_clicks"),
#     State("draw-line-mode", "data"),
#     State("draw-method-modal", "is_open"),
#     prevent_initial_call=True,
# )
# def open_draw_method_modal(n_clicks, draw_line_mode, is_open):
#     ctx = callback_context
#     if not ctx.triggered:
#         return no_update

#     button_id = ctx.triggered[0]['prop_id'].split('.')[0]
#     if button_id == 'draw-line-button' and draw_line_mode:  # If the draw-line button was clicked and draw line mode is already on.
#         return False  #Prevent modal open
#     return not is_open


# # Callback to set the draw method (manual or coordinate) and close the modal
# @callback(
#     Output("draw-method", "data"),
#     Output("draw-method-modal", "is_open", allow_duplicate=True),
#     Input("manual-draw-button", "n_clicks"),
#     Input("coordinate-draw-button", "n_clicks"),
#     State("draw-method-modal", "is_open"),
#     prevent_initial_call=True,
# )
# def set_draw_method(manual_clicks, coordinate_clicks, is_open):
#     ctx = callback_context
#     if not ctx.triggered:
#         return no_update, no_update
#     button_id = ctx.triggered[0]['prop_id'].split('.')[0]
#     if button_id == "manual-draw-button":
#         return "manual", False  # Chọn manual và đóng modal
#     elif button_id == "coordinate-draw-button":
#         return "coordinate", False  # Chọn coordinate và đóng modal
#     return "", is_open

# # Callback to open coordinate modal when "coordinate" draw method is selected
# @callback(
#     Output("coordinate-modal", "is_open"),
#     Input("draw-method", "data"),
#     State("coordinate-modal", "is_open"),
#     prevent_initial_call=True,
# )
# def open_coordinate_modal(draw_method, is_open):
#     if draw_method == "coordinate":
#         return True
#     return False

# # Callback to draw line based on coordinates entered in the modal
# @callback(
#     Output("map-image-draw-mode", "figure"),
#     Input("draw-button", "n_clicks"),
#     State("start-x", "value"),
#     State("start-y", "value"),
#     State("end-x", "value"),
#     State("end-y", "value"),
#     State("map-image-draw-mode", "figure"),
#     prevent_initial_call=True,
# )
# def draw_line_coordinate(n_clicks, start_x, start_y, end_x, end_y, figure):
#     if n_clicks is None:
#         return no_update

#     try:
#         start_x = float(start_x)
#         start_y = float(start_y)
#         end_x = float(end_x)
#         end_y = float(end_y)
#     except (ValueError, TypeError):
#         print("Invalid coordinates entered.")  # Log the error
#         return figure  # Return the figure unchanged

#     # Add line to the figure
#     figure["data"].append(
#         go.Scatter(
#             x=[start_x, end_x],
#             y=[start_y, end_y],
#             mode="lines",
#             line=dict(color="blue", width=2),
#             showlegend=False,
#         )
#     )
#     return figure

# # Callback to store the start point when the user clicks on the graph
# @callback(
#     Output("line-coordinates", "data"),
#     Input("map-image-draw-mode", "clickData"),
#     State("draw-method", "data"),
#     State("draw-line-mode", "data"), # Add draw_line_mode to State
#     prevent_initial_call=True,
# )
# def store_start_point(clickData, draw_method, draw_line_mode): # Add draw_line_mode
#     print("store_start_point called")  # Add this line
#     print("draw_method:", draw_method)  # Add this line
#     print("clickData:", clickData) # Add this line
#     print("draw_line_mode:", draw_line_mode) # add this line

#     if draw_method == "manual" and clickData and draw_line_mode: # check if draw_line_mode is True
#         x = clickData["points"][0]["x"]
#         y = clickData["points"][0]["y"]
#         return {"start_x": x, "start_y": y}
#     return {}

# # Callback to draw the line when the user releases the mouse (relayoutData)
# @callback(
#     Output("map-image-draw-mode", "figure", allow_duplicate=True),
#     Input("map-image-draw-mode", "relayoutData"),
#     Input("draw-method", "data"),  # Add draw-method as an Input
#     State("line-coordinates", "data"),
#     State("map-image-draw-mode", "figure"),
#      State("draw-line-mode", "data"),  # Add draw_line_mode as State

#     prevent_initial_call=True,
# )
# def draw_line_on_release(relayoutData,draw_method, line_coordinates, figure, draw_line_mode): # add draw_line_mode to args
#     print("draw_line_on_release called")
#     print("draw_method:", draw_method)
#     print("relayoutData:", json.dumps(relayoutData))  # Print relayoutData
#     print("line_coordinates:", line_coordinates) # Add this line
#     print("draw_line_mode:", draw_line_mode)  #print draw line mode


#     if draw_method == "manual" and relayoutData and line_coordinates and "start_x" in line_coordinates and draw_line_mode: # and check for draw line mode
#         # Check if relayoutData is due to a drag/release (mouse up)
#         if 'xaxis.range[0]' in relayoutData and 'yaxis.range[0]' in relayoutData and 'xaxis.range[1]' in relayoutData and 'yaxis.range[1]' in relayoutData:
#             # Extract start point from stored coordinates
#             start_x = line_coordinates["start_x"]
#             start_y = line_coordinates["start_y"]

#             # Extract end point from relayoutData (mouse release)
#             end_x = relayoutData['xaxis.range[1]']
#             end_y = relayoutData['yaxis.range[0]']

#             # Add line to the figure
#             figure["data"].append(
#                 go.Scatter(
#                     x=[start_x, end_x],
#                     y=[start_y, end_y],
#                     mode="lines",
#                     line=dict(color="green", width=2),  # Changed color to green
#                     showlegend=False,
#                 )
#             )
#             return figure
#         else:
#             return no_update
#     else:
#         return no_update

# # Callback to clear the stored start point after drawing
# @callback(
#     Output("line-coordinates", "data", allow_duplicate=True),
#     Input("map-image-draw-mode", "figure"),
#     prevent_initial_call=True,
# )
# def clear_start_point(figure):
#     return {}

# @callback(
#     Output("draw-line-mode", "data"),
#     Input("draw-line-button", "n_clicks"),
#     State("draw-line-mode", "data"),
#     prevent_initial_call=True,
# )
# def toggle_draw_line_mode(n_clicks, current_state):
#     return not current_state

# @callback(
#     Output("map-image-draw-mode", "dragmode"),
#     Input("draw-line-mode", "data"),
#     prevent_initial_call=True,
# )
# def update_drag_mode(draw_line_mode):
#     if draw_line_mode:
#         return "drawline"  # Cho phép vẽ khi chế độ vẽ được kích hoạt
#     else:
#         return "pan"  # Ngăn người dùng vẽ khi chế độ vẽ không được kích hoạt

# @callback(
#     Output("draw-line-button", "style"),
#      Input("draw-line-mode", "data"),
#      State("button-style-store", "data")
# )
# def update_button_style(is_active, button_style_store):
#     default_style = button_style_store["draw_line_button"]
#     active_button_style = {
#         "padding": "8px 16px",
#         "border": "1px solid #2ecc71",  # A different color for active state
#         "color": "white",
#         "background-color": "#2ecc71",  # A different background color for active state
#         "border-radius": "5px",
#         "transition": "all 0.3s ease-in-out",
#         "cursor": "pointer",
#     }
#     if is_active:
#         return active_button_style
#     else:
#         return default_style
    