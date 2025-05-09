import rospy
import tf
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image, ImageDraw
import math

# Constants
OUTPUT_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/combined_image.png"
MAP_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/map_image.png"  # Correct path here
ROBOT_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/robot_image.png"
PATH_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/path_image.png"
F_SCAN_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/f_scan_image.png"
B_SCAN_IMAGE_PATH = "/home/duc/Downloads/App MIR100/static/b_scan_image.png"
IMAGE_WIDTH = None
IMAGE_HEIGHT = None
MAP_ORIGIN_X = None
MAP_ORIGIN_Y = None
MAP_RESOLUTION = None
LIDAR_RANGE = 50  # Maximum range of the LiDAR to consider
POINT_SIZE = 1

# Function to convert world coordinates to image coordinates
def world_to_image(x, y):
    global IMAGE_WIDTH, IMAGE_HEIGHT, MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION
    if MAP_ORIGIN_X is None or MAP_ORIGIN_Y is None or MAP_RESOLUTION is None or IMAGE_WIDTH is None or IMAGE_HEIGHT is None:
        rospy.logwarn("Map information not yet received. Cannot transform coordinates.")
        return None, None

    px = int((x - MAP_ORIGIN_X) / MAP_RESOLUTION)
    py = int(IMAGE_HEIGHT - (y - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return px, py


def process_lidar_data(msg, tf_listener, frame_id):
    points = []

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min

        if msg.range_min < r < msg.range_max and r < LIDAR_RANGE:
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            point_stamped = geometry_msgs.msg.PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = msg.header.stamp
            point_stamped.point.x = x
            point_stamped.point.y = y

            try:
                tf_listener.waitForTransform("map", frame_id, msg.header.stamp, rospy.Duration(1))
                transformed_point = tf_listener.transformPoint("map", point_stamped)
                points.append((transformed_point.point.x, transformed_point.point.y))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF error: {e}")
                continue
    return points


def create_lidar_image(points):
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    if not points:
        return img
    for point_x, point_y in points:
        px, py = world_to_image(point_x, point_y)

        if px is not None and py is not None:
            draw.ellipse((px - POINT_SIZE, py - POINT_SIZE, px + POINT_SIZE, py + POINT_SIZE), fill=(255, 0, 0))

    return img


def path_callback(msg):
    global IMAGE_WIDTH, IMAGE_HEIGHT
    try:
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        for world_x, world_y in path_points:
            px, py = world_to_image(world_x, world_y)
            if px is not None and py is not None:
                draw.ellipse((px - 0.5, py - 0.5, px + 0.5, py + 0.5), fill=(0, 255, 255))  # Slightly larger path points

        img.save(PATH_IMAGE_PATH)
        rospy.loginfo(f"Path image saved to {PATH_IMAGE_PATH}")

    except Exception as e:
        rospy.logerr(f"Error processing path data: {e}")

def pose_callback(msg, tf_listener):
    global IMAGE_WIDTH, IMAGE_HEIGHT
    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        x, y = trans[0], trans[1]
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)

        img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        px, py = world_to_image(x, y)

        rect_length = 0.8
        rect_width = 0.6
        tri_side = 0.3

        corners_world = [
            (x + rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
             y + rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw)),
            (x + rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
             y + rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
            (x - rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
             y - rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
            (x - rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
             y - rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw))
        ]
        corners_image = [world_to_image(corner[0], corner[1]) for corner in corners_world]
        if all(corner is not None for corner in corners_image):
          draw.polygon(corners_image, fill=(128, 128, 128, 102))

        triangle_points_world = [
            (x + tri_side * math.cos(yaw), y + tri_side * math.sin(yaw)),
            (x - tri_side / 2 * math.cos(yaw) + (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
             y - tri_side / 2 * math.sin(yaw) - (tri_side * math.sqrt(3) / 2) * math.cos(yaw)),
            (x - tri_side / 2 * math.cos(yaw) - (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
             y - tri_side / 2 * math.sin(yaw) + (tri_side * math.sqrt(3) / 2) * math.cos(yaw))
        ]

        triangle_points_image = [world_to_image(point[0], point[1]) for point in triangle_points_world]

        if all(point is not None for point in triangle_points_image):
          draw.polygon(triangle_points_image, fill=(0, 0, 255, 178))
        img.save(ROBOT_IMAGE_PATH)
        rospy.loginfo(f"Robot location image updated: {ROBOT_IMAGE_PATH}")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
       rospy.logerr(f"TF error: {e}")
    except Exception as e:
        rospy.logerr(f"There was an error in displaying robot {e}")


def map_info_callback(map_data):
    """Callback function to handle map information."""
    global MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION, IMAGE_WIDTH, IMAGE_HEIGHT

    MAP_ORIGIN_X = map_data.info.origin.position.x
    MAP_ORIGIN_Y = map_data.info.origin.position.y
    MAP_RESOLUTION = map_data.info.resolution
    IMAGE_WIDTH = map_data.info.width
    IMAGE_HEIGHT = map_data.info.height
    rospy.loginfo("Received map info")


def lidar_callback(msg, tf_listener, topic_name):
    """Callback function for lidar data."""
    try:
        frame_id = "back_laser_link" if topic_name == "/b_scan" else "front_laser_link"
        points = process_lidar_data(msg, tf_listener, frame_id)
        img_output = create_lidar_image(points)
        output_path = f"/home/duc/Downloads/App MIR100/static/{topic_name.split('/')[-1]}_image.png"
        img_output.save(output_path)
        rospy.loginfo(f"Lidar image for {topic_name} created and saved to {output_path}")
    except Exception as e:
        rospy.logerr(f"Error processing {topic_name} data: {e}")

def combine_images():
    """Combines the map, robot, path and lidar images into one."""
    try:
        # Load the images
        map_img = Image.open(MAP_IMAGE_PATH).convert("RGBA")  # Ensure it's RGBA
        robot_img = Image.open(ROBOT_IMAGE_PATH).convert("RGBA")
        path_img = Image.open(PATH_IMAGE_PATH).convert("RGBA")
        f_scan_img = Image.open(F_SCAN_IMAGE_PATH).convert("RGBA")
        b_scan_img = Image.open(B_SCAN_IMAGE_PATH).convert("RGBA")

        # Paste the images onto the map image
        map_img.paste(path_img, (0, 0), path_img)
        map_img.paste(robot_img, (0, 0), robot_img)
        map_img.paste(f_scan_img, (0, 0), f_scan_img)
        map_img.paste(b_scan_img, (0, 0), b_scan_img)


        # Save the combined image
        map_img.save(OUTPUT_IMAGE_PATH)
        rospy.loginfo(f"Combined image saved to {OUTPUT_IMAGE_PATH}")

    except FileNotFoundError as e:
        rospy.logerr(f"One or more image files not found: {e}")
    except Exception as e:
        rospy.logerr(f"Error combining images: {e}")

def main():
    """Main function."""
    rospy.init_node('image_combiner_node', anonymous=True)
    tf_listener = tf.TransformListener()

    # Subscribers
    rospy.Subscriber("/map", OccupancyGrid, map_info_callback)
    rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, path_callback)
    rospy.Subscriber("/f_scan", LaserScan, lambda msg: lidar_callback(msg, tf_listener, "/f_scan"))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: lidar_callback(msg, tf_listener, "/b_scan"))
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, lambda msg: pose_callback(msg, tf_listener))

    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        combine_images()  
        rate.sleep()

if __name__ == '__main__':
    try:
        import geometry_msgs.msg
        main()
    except rospy.ROSInterruptException:
        pass