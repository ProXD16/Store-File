import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import io
import yaml
import json

def map_callback(map_data):
    try:
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        resolution_x = resolution  # Assign resolution to resolution_x
        resolution_y = resolution  # Assign resolution to resolution_y
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        map_array = np.array(map_data.data).reshape((height, width))
        map_array = np.flipud(map_array)
        map_array = np.rot90(map_array, k=4)
        map_array = (map_array * 255 / 100).astype(np.uint8)
        map_array = 255 - map_array
        img = Image.fromarray(map_array, mode='L')
        #img = img.resize((600, 800), Image.LANCZOS) #resize the image by LANCZO
        buffer = io.BytesIO()
        img.save(buffer, format="png")
        encoded_image = buffer.getvalue()
        
        image_path = "/home/duc/Downloads/App MIR100/static/map_image.png"
        yaml_path = "/home/duc/Downloads/App MIR100/static/map_image.yaml"
        info_path = "/home/duc/Downloads/App MIR100/static/map_image.info" # New path for map info
        json_path = "/home/duc/Downloads/App MIR100/static/map_image.json"  # Path for JSON file
        
        with open(image_path, "wb") as fh:
            fh.write(encoded_image)

        rospy.loginfo(f"Saved rotated map image to {image_path}")

        # Save YAML file
        map_yaml = {
            'image': 'map_image.png',
            'resolution': resolution,
            'origin': [origin_x, origin_y, 0.0],  # Assuming no rotation
            'negate': 0,  # As we've already negated the values
            'occupied_thresh': 0.65, #These two values should probably be fine-tuned
            'free_thresh': 0.196
        }

        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(map_yaml, yaml_file)

        rospy.loginfo(f"Saved map YAML data to {yaml_path}")

        # Save additional map information (YAML)
        map_info = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'resolution_x': resolution_x,  # Add resolution_x
            'resolution_y': resolution_y,  # Add resolution_y
            'origin_x': origin_x,
            'origin_y': origin_y
            # Add any other relevant information here
        }

        with open(info_path, 'w') as info_file:
            yaml.dump(map_info, info_file)

        rospy.loginfo(f"Saved map info data to {info_path}")


        # Save additional map information (JSON)
        map_json = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'resolution_x': resolution_x, # Add resolution_x to JSON
            'resolution_y': resolution_y, # Add resolution_y to JSON
            'origin_x': origin_x,
            'origin_y': origin_y
            # Add any other relevant information here
        }

        with open(json_path, 'w') as json_file:
            json.dump(map_json, json_file)  # Use json.dump to save as JSON

        rospy.loginfo(f"Saved map info data to {json_path}")

    except Exception as e:
        rospy.logerr(f"Error processing map data: {e}")

def listener():
    rospy.init_node('map_to_image', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()