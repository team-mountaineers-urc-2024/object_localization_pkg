import rclpy
from rclpy.node import Node
import rclpy.time
from ros2_aruco_interfaces.msg import ArucoMarkers
from robot_interfaces.srv import ObjectPosition
from robot_interfaces.msg import UrcCustomPoint, UrcCustomPath
from builtin_interfaces.msg import Time, Duration
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PointStamped as tf2_PointStamped
from geometry_msgs.msg import PointStamped as geo_PointStamped
from tf2_geometry_msgs import do_transform_point

# TODO: Determine Magic Functions for Left, Right, and Back
# TODO: Determine Magic Function for Z axis

class ObjectLocalizationNode(Node):

    def __init__(self):
        super().__init__('object_localization_node')

        self.declare_parameter('subscription_topics', ['/logitech_01/aruco_markers', '/logitech_03/aruco_markers', '/logitech_05/aruco_markers', '/logitech_10/aruco_markers'])  
        self.declare_parameter('global_origin_frame', 'global_origin')
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('aruco_magic_number', 0.489)
        
        # Set up the marker service
        self.position_srv = self.create_service(ObjectPosition, 'get_aruco_positions', self.position_callback)

        self.subscription_topics = self.get_parameter('subscription_topics').get_parameter_value().string_array_value
        self.subscribers = []
        self.seen_markers = [[]]*len(self.subscription_topics)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Make a list of all the wanted subscribers
        for topic in self.subscription_topics:
            self.subscribers.append(
                self.create_subscription(
                    ArucoMarkers,
                    topic,
                    lambda msg,topic=topic: self.pose_callback(msg, topic),
                    10
                )
            )

        self.point_pub = self.create_publisher(
            tf2_PointStamped,
            '/markers',
            10
        )

    # Return the list of known aruco marker points as a service
    def position_callback(self, request, response):
        
        # Create the response message
        response.bag = UrcCustomPath()
        curr_index = 0
        marker_0_index = -1
        marker_1_index = -1
        marker_2_index = -1
        marker_3_index = -1
        marker_4_index = -1
        marker_5_index = -1

        # Loop through the 2D array and add it to the response message
        for marker_list in self.seen_markers:

            for marker in marker_list:

                # Check if the marker already exists
                match (marker.location_label):


                    # If the current marker seen is marker 0
                    case "aruco_marker_0":

                        # If it already exists
                        if (marker_0_index != -1):
                            
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_0_index].point.header.stamp)):
                                response.bag.points[marker_0_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_0_index = curr_index
                            curr_index += 1

                    # If the current marker seen is marker 1
                    case "aruco_marker_1":
                        # If it already exists
                        if (marker_1_index != -1):
                            
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_1_index].point.header.stamp)):
                                response.bag.points[marker_1_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_1_index = curr_index
                            curr_index += 1

                    # If the current marker seen is marker 2
                    case "aruco_marker_2":
                        # If it already exists
                        if (marker_2_index != -1):
        
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_2_index].point.header.stamp)):
                                response.bag.points[marker_2_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_2_index = curr_index
                            curr_index += 1

                    # If the current marker seen is marker 3
                    case "aruco_marker_3":
                        # If it already exists
                        if (marker_3_index != -1):
                            
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_3_index].point.header.stamp)):
                                response.bag.points[marker_3_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_3_index = curr_index
                            curr_index += 1

                    # If the current marker seen is marker 4
                    case "aruco_marker_4":
                        # If it already exists
                        if (marker_4_index != -1):
                            
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_4_index].point.header.stamp)):
                                response.bag.points[marker_4_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_4_index = curr_index
                            curr_index += 1

                    # If the current marker seen is marker 5
                    case "aruco_marker_5":
                        # If it already exists
                        if (marker_5_index != -1):
                            
                            # Check to see which is newer
                            if (self.float_time_convert(marker.point.header.stamp) > self.float_time_convert(response.bag.points[marker_5_index].point.header.stamp)):
                                response.bag.points[marker_5_index] = marker

                        # If it does not exist
                        else:
                            response.bag.points.append(marker)
                            marker_5_index = curr_index
                            curr_index += 1

                    # Default case
                    case _:
                        self.get_logger().error(f"Unknown marker: {marker.location_label}")

        response.bag.time_recieved = self.get_clock().now().to_msg()
        return response

    # Update the current list of known aruco markers
    def pose_callback(self, msg, source):

        camera_spec_markers = []

        # Get the length of the pose array
        length = len(msg.marker_ids)
        camera_frame = msg.header.frame_id
        tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        magic_number = self.get_parameter('aruco_magic_number').get_parameter_value().double_value

        global_frame = self.get_parameter('global_origin_frame').get_parameter_value().string_value

        # Get the transform the the current camera frame
        try:
            self.tf_buffer.lookup_transform_async
            zero = Time()
            zero.sec = 0
            zero.nanosec = 0
            camera_tf = self.tf_buffer.lookup_transform(
                camera_frame,
                self.get_parameter('global_origin_frame').get_parameter_value().string_value,
                # rclpy.time.Time(0) # 
                zero
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {camera_frame} to {global_frame}: {ex}')
            return


        for i in range(0, length):
            # self.get_logger().info(f"{source}")
            # Get the pose
            pose = msg.poses[i]

            temp_point = tf2_PointStamped()
            # TODO: Make fix plz
            # Works because transforms are werid
            temp_point.point.y = pose.position.x#magic_function(pose.position.x, source)
            temp_point.point.x = -pose.position.y#magic_function(-pose.position.y, source)
            temp_point.point.z = pose.position.z#magic_function(pose.position.z, source)
            temp_point.header.frame_id = camera_frame
            temp_point.header.stamp = camera_tf.header.stamp
            # self.tf_buffer.lookup_transform

            # transformed_point = self.
            transformed_point = self.tf_buffer.transform(temp_point, global_frame)
            # self.get_logger().info(transformed_point)

            # Testing point
            testing_point = tf2_PointStamped()
            testing_point.point.x = transformed_point.point.x
            testing_point.point.y = transformed_point.point.y
            # TODO
            testing_point.point.z = transformed_point.point.z
            testing_point.header.frame_id = global_frame
            testing_point.header.stamp = transformed_point.header.stamp

            self.point_pub.publish(testing_point)


            # Calculate the point
            marker_point = UrcCustomPoint()
            marker_point.point.point.x = transformed_point.point.x
            marker_point.point.point.y = transformed_point.point.y
            # TODO
            marker_point.point.point.z = transformed_point.point.z
            marker_point.point.header.frame_id = transformed_point.header.frame_id
            marker_point.point.header.stamp = transformed_point.header.stamp
            marker_point.location_label = f"aruco_marker_{msg.marker_ids[i]}"

            # Append this marker to the list of ones we see right now
            camera_spec_markers.append(marker_point)
        
        # Replace the last group of markers this camera saw with the current one
        self.seen_markers[self.subscription_topics.index(source)] = camera_spec_markers

    # Return a one-variable version of the ROS Time message
    def float_time_convert(self, time : Time):
        return time.sec + time.nanosec * 10**(-9)
    
# The values for this function were determined empirically through testing and lots of desmos graphs
def magic_function(x, source):
    # # If the source was the Front Camera
    # if (source == '/logitech_05/aruco_markers'):
    #     return 0.338987*x# + 0.320118
    
    # # If the source was the Left Camera
    # # TODO
    # elif (source == '/logitech_01/aruco_markers'):
    #     return 0.314458617541*x# + 0.3834383
    
    # # If the source was the Right Camera
    # # TODO
    # elif (source == '/logitech_03/aruco_markers'):
    #     return 0.307973*x# - 0.320981
    
    # # If the source was the Back Camera
    # # TODO
    # elif (source == '/logitech_10/aruco_markers'):
    #     return 0.331000448238*x# + 0.123349
    
    # # Otherwise
    # else:
    #     return x
    return x

def main(args=None):
    rclpy.init(args=args)

    localization_node = ObjectLocalizationNode()

    rclpy.spin(localization_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()