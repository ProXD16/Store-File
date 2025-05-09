import rospy
import nav_msgs.msg
import yaml
import os
import numpy as np

class MapManager:
    def __init__(self, save_directory="saved_maps"):
        self.save_directory = save_directory
        os.makedirs(self.save_directory, exist_ok=True)  # Create directory if it doesn't exist
        self.map_subscriber = None
        self.current_map_data = None

    def start_map_capture(self):
        """Starts subscribing to the /map topic to capture the map."""
        if self.map_subscriber is None:
            self.map_subscriber = rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.map_callback)
            rospy.loginfo("Map capture started. Subscribing to /map topic.")
        else:
            rospy.logwarn("Map capture already in progress.")

    def stop_map_capture(self):
        """Stops subscribing to the /map topic."""
        if self.map_subscriber is not None:
            self.map_subscriber.unregister()
            self.map_subscriber = None
            rospy.loginfo("Map capture stopped.")
        else:
            rospy.logwarn("Map capture not in progress.")

    def map_callback(self, data):
        """Callback function to receive map data."""
        self.current_map_data = data

    def save_map(self, map_name):
        """Saves the captured map to a YAML file."""
        if self.current_map_data is not None:
            map_data = self.current_map_data
            file_path = os.path.join(self.save_directory, f"{map_name}.yaml")
            try:
                # Prepare data for YAML saving
                map_info = {
                    'image': f"{map_name}.pgm",  # Placeholder, PGM saving is handled separately
                    'resolution': map_data.info.resolution,
                    'origin': [map_data.info.origin.position.x,
                               map_data.info.origin.position.y, 0.0],
                    'negate': 0,
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.196
                }

                # Save YAML file
                with open(file_path, 'w') as yaml_file:
                    yaml.dump(map_info, yaml_file, default_flow_style=False)

                # Save PGM image file
                image_path = os.path.join(self.save_directory, f"{map_name}.pgm")
                self.save_occupancy_grid_as_pgm(map_data, image_path)

                rospy.loginfo(f"Map saved to {file_path} and {image_path}")
            except Exception as e:
                rospy.logerr(f"Error saving map: {e}")
        else:
            rospy.logwarn("No map data available to save. Please capture a map first.")

    def save_occupancy_grid_as_pgm(self, occupancy_grid, filename):
        """Saves OccupancyGrid data to a PGM image file."""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y

        # Convert occupancy grid data to a NumPy array
        data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))

        # PGM files need values between 0-255.  Scale the occupancy grid values.
        # Occupied cells will be 0, free cells will be 255, unknown will be 128.
        image_data = np.zeros((height, width), dtype=np.uint8)
        image_data[data == 0] = 255  # Free space
        image_data[data == 100] = 0  # Occupied space
        image_data[data == -1] = 128  # Unknown space

        try:
            with open(filename, 'w') as pgm_file:
                # Write PGM header
                pgm_file.write('P5\n')  # Magic number for binary PGM
                pgm_file.write(f'{width} {height}\n')  # Width and height
                pgm_file.write('255\n')  # Maximum gray value

                # Write image data
                image_data.T.tofile(pgm_file)  # Transpose to match ROS's row-major order
            rospy.loginfo(f"Successfully saved PGM image to {filename}")

        except Exception as e:
            rospy.logerr(f"Error saving PGM image: {e}")
            
    def load_map(self, map_name):
        """Loads a map from a YAML file and returns the OccupancyGrid message."""
        file_path = os.path.join(self.save_directory, f"{map_name}.yaml")
        image_path = os.path.join(self.save_directory, f"{map_name}.pgm")

        try:
            # Load YAML file
            with open(file_path, 'r') as yaml_file:
                map_info = yaml.safe_load(yaml_file)

            # Load PGM image
            image_data = self.load_pgm_image(image_path)
            
            # Create OccupancyGrid message
            occupancy_grid = nav_msgs.msg.OccupancyGrid()
            occupancy_grid.info.resolution = map_info['resolution']
            occupancy_grid.info.width = image_data.shape[1]
            occupancy_grid.info.height = image_data.shape[0]
            occupancy_grid.info.origin.position.x = map_info['origin'][0]
            occupancy_grid.info.origin.position.y = map_info['origin'][1]
            occupancy_grid.info.origin.position.z = 0.0

            # Convert image data to OccupancyGrid data (0, 100, -1)
            data = np.zeros((image_data.shape[0], image_data.shape[1]), dtype=np.int8)
            data[image_data == 255] = 0  # Free space
            data[image_data == 0] = 100  # Occupied space
            data[image_data == 128] = -1  # Unknown space

            occupancy_grid.data = data.flatten().tolist()

            rospy.loginfo(f"Successfully loaded map from {file_path} and {image_path}")
            return occupancy_grid

        except FileNotFoundError:
            rospy.logerr(f"Map file not found: {file_path}")
            return None
        except Exception as e:
            rospy.logerr(f"Error loading map: {e}")
            return None
            
    def load_pgm_image(self, filename):
        """Loads a PGM image and returns the image data as a NumPy array."""
        try:
            with open(filename, 'rb') as pgm_file:
                # Read PGM header
                header = pgm_file.readline().decode('utf-8').strip()
                if header != 'P5':
                    raise ValueError("Invalid PGM format. Only P5 (binary) is supported.")
                size_line = pgm_file.readline().decode('utf-8').strip()
                width, height = map(int, size_line.split())
                max_val = int(pgm_file.readline().decode('utf-8').strip())

                # Read image data
                image_data = np.fromfile(pgm_file, dtype=np.uint8).reshape((height, width))
                return image_data

        except FileNotFoundError:
            rospy.logerr(f"PGM image file not found: {filename}")
            return None
        except Exception as e:
            rospy.logerr(f"Error loading PGM image: {e}")
            return None