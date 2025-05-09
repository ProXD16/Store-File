import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

def draw_arc(ax, start_point, end_point, radius, color='blue'):
    """
    Draws a circular arc given two points on the arc and the radius.

    Args:
        ax: Matplotlib axes object to draw on.
        start_point: (x, y) tuple of the starting point of the arc.
        end_point: (x, y) tuple of the ending point of the arc.
        radius: Radius of the circle.
        color: Color of the arc.
    """
    x1, y1 = start_point
    x2, y2 = end_point

    #Calculate distance
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    #Check if there is intersection.
    if distance > 2 *radius:
        return #Invalid Request.

    #Calculate the center of the circle
    mid_x = (x1 + x2)/2
    mid_y = (y1 + y2)/2

    #Calculate offset (perpendicular slope)
    offset = math.sqrt(radius**2 - (distance/2)**2)

    #Find orientation for offset
    orientationAngle = math.atan2((y2 - y1),(x2-x1))

    #Apply the offset
    center_x = mid_x - offset * math.sin(orientationAngle)
    center_y = mid_y + offset * math.cos(orientationAngle)

    #Calculate Start and End Angle
    startAngle = math.atan2(y1- center_y, x1 - center_x) *180 / math.pi
    endAngle = math.atan2(y2- center_y, x2 - center_x) *180 / math.pi

    #Draw arc
    arc = patches.Arc((center_x, center_y), width= radius*2, height = radius*2 , angle = 0, theta1 = startAngle, theta2 = endAngle, color = color)
    ax.add_patch(arc)

if __name__ == '__main__':
    fig, ax = plt.subplots()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')  # Ensure circles look like circles

    start_point = (1, 1)
    end_point = (3, 2)
    radius = 2
    draw_arc(ax, start_point, end_point, radius)

    plt.show()