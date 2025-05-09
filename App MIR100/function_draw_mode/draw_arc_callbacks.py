# arc_drawing.py
from dash import html, dcc, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import dash
import numpy as np

def draw_arc(center_x, center_y, radius, start_angle, end_angle, color='blue', width=2):
    """
    Draws an arc.
    """
    num_points = 100
    angles = np.linspace(start_angle, end_angle, num_points)
    angles_rad = np.deg2rad(angles)
    x = center_x + radius * np.cos(angles_rad)
    y = center_y + radius * np.sin(angles_rad)
    arc = go.Scatter(
        x=x,
        y=y,
        mode='lines',
        line=dict(color=color, width=width),
        hoverinfo='none',
        showlegend=False,
    )
    return arc

def create_arc_modal():
    """
    Creates modal for entering arc information.
    """
    arc_modal = dbc.Modal(
        [
            dbc.ModalHeader(dbc.ModalTitle("Enter Arc Information")),
            dbc.ModalBody(
                [
                    dbc.Label("Center X:"),
                    dbc.Input(type="number", id="arc-center-x"),
                    dbc.Label("Center Y:"),
                    dbc.Input(type="number", id="arc-center-y"),
                    dbc.Label("Radius:"),
                    dbc.Input(type="number", id="arc-radius"),
                    dbc.Label("Start Angle (degrees):"),
                    dbc.Input(type="number", id="arc-start-angle"),
                    dbc.Label("End Angle (degrees):"),
                    dbc.Input(type="number", id="arc-end-angle"),
                ]
            ),
            dbc.ModalFooter(
                [
                    dbc.Button("Draw Arc", id="draw-arc-button-modal", color="primary"),
                    dbc.Button("Cancel", id="cancel-arc-button"),
                ]
            ),
        ],
        id="arc-modal",
        is_open=False,
    )
    return arc_modal


def create_arc_callbacks(app):
    """
    Creates callbacks related to arc drawing.
    """
    # Show the arc input modal when "Draw Arc" is clicked.
    @app.callback(
        Output("arc-modal", "is_open"),
        Input("draw-arc-button", "n_clicks"),
        Input("cancel-arc-button", "n_clicks"),
        State("arc-modal", "is_open"),
        prevent_initial_call=False
    )
    def toggle_arc_modal(n_clicks_draw, n_clicks_cancel, is_open):
      ctx = dash.callback_context
      if not ctx.triggered:
          return False

      trigger_id = ctx.triggered[0]["prop_id"].split(".")[0]

      if trigger_id == "draw-arc-button":
          return not is_open
      elif trigger_id == "cancel-arc-button":
          return False
      return is_open
    @app.callback(
    Output("map-image-draw-mode", "figure"),
    [Input("draw-arc-button-modal", "n_clicks")],
    [
        State("arc-center-x", "value"),
        State("arc-center-y", "value"),
        State("arc-radius", "value"),
        State("arc-start-angle", "value"),
        State("arc-end-angle", "value"),
        State("map-image-draw-mode", "figure"),
        State("saved-arcs", "data")  # Added state for saved_arcs

    ],
    prevent_initial_call=True,
    )
    def draw_arc_button(n_clicks, center_x, center_y, radius, start_angle, end_angle, figure,saved_arcs):
      if n_clicks is None:
          return figure

      try:
          center_x = float(center_x)
          center_y = float(center_y)
          radius = float(radius)
          start_angle = float(start_angle)
          end_angle = float(end_angle)

          arc = draw_arc(center_x, center_y, radius, start_angle, end_angle)

          # Append new arc trace to the figure's data
          figure["data"] = figure["data"] + (arc,)
          return figure
      except Exception as e:
        print(f"Error drawing arc: {e}")
        return figure