import rclpy
from collections import deque
from rclpy.node import Node
from rclpy.time  import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage
from message_filters import Subscriber, ApproximateTimeSynchronizer
import argparse, csv, os, time


DECISION_MARGIN_THRESHOLD = 80.0  # Adjust as needed

class ImageSubscriber(Node):
    def __init__(self, freq = 5.0, output='test1'):
        super().__init__('odom_throttler')

        self.subscriber_tag = self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.tag_callback, 10)
        self.subscriber_tag_tf = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        self.timer = self.create_timer(0.01, self.write_to_csv)

        # newpath = output 
        # if not os.path.exists(newpath):
        #     os.makedirs(newpath)

        self.csv_file = open(output + '.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # store tags in deques for storing, then synchronizing
        self.tf_buffer = deque()   #Buffer of TFMessage, a list of TransformStamped
        self.tag_buffer = deque()  #Buffer of AprilTagDetectionArray, a header + list of AprilTagDetection

        ##########
        # Throttle settings
        self.last_time = self.get_clock().now()
        self.throttle_period = 1.0 / freq  # 5Hz
        
    # Subscribe to /apriltag/detections and add all tags to buffer if length > 0 (tags detected)
    def tag_callback(self, msg: AprilTagDetectionArray):
        detections = msg.detections
        if len(detections) > 0:
            self.tag_buffer.append(msg)

    # Subscribe to /tf and add all tags to buffer if length > 0 (tags detected)
    def tf_callback(self, msg: TFMessage):
        if len(msg.transforms) > 0:
            self.tf_buffer.append(msg.transforms)
            
    # Periodically write to csv file by going through the buffers
    def write_to_csv(self):
        while self.tf_buffer and self.tag_buffer:
            tf_msg = self.tf_buffer[0]
            tag_msg = self.tag_buffer[0]

            #Match timestamps for each batch of tf and tag messages
            #For tf, each transform in the list has the same timestamp, so use the first one
            #For tag, one header is provided for the entire array of detections
            if tf_msg[0].header.stamp.sec == tag_msg.header.stamp.sec and \
               tf_msg[0].header.stamp.nanosec == tag_msg.header.stamp.nanosec:
                print(f"Found time match at: {tf_msg[0].header.stamp.sec + tf_msg[0].header.stamp.nanosec * 1e-9}")
                tag_detections = [tag_id.id for tag_id in tag_msg.detections]  # Get list of tag IDs from detections
                # Loop over each detected tag with this timestamp
                for tf in tf_msg:
                    # Get the tag ID from the child frame ID and find corresponding index in detections
                    index = -1
                    tf_id = int(tf.child_frame_id.split(':')[-1])
                    try:
                        index = tag_detections.index(tf_id)
                        #print(f"Tag {tf.child_frame_id} found in detections at index {index}")
                    except:
                        self.get_logger().error(f"Tag {tf.child_frame_id} not found in detections")
                        continue
                    #match found
                    if index >= 0:
                        #Check if decision margin is above threshold
                        if tag_msg.detections[index].decision_margin > DECISION_MARGIN_THRESHOLD:
                            # Write to CSV
                            self.get_logger().info(f"Writing TF for tag {tf.child_frame_id} with decision margin {tag_msg.detections[index].decision_margin:.2f}")
                            t = tf.transform.translation
                            q = tf.transform.rotation
                            self.csv_writer.writerow([
                                tf.header.stamp.sec, tf.header.stamp.nanosec,
                                tf.header.frame_id,
                                tf.child_frame_id,
                                t.x, t.y, t.z,
                                q.x, q.y, q.z, q.w
                            ])

                        else:
                            self.get_logger().warn(f"Tag {tf.child_frame_id} has low decision margin: {tag_msg.detections[index].decision_margin:.2f}")
                            continue

                # Remove from buffers
                self.tf_buffer.popleft()
                self.tag_buffer.popleft()

            # Drop old unmatched entries
            # If first tf has older timestamp than tag, remove tf
            elif tf_msg[0].header.stamp.sec + tf_msg[0].header.stamp.nanosec * 1e-9 < \
                tag_msg.header.stamp.sec + tag_msg.header.stamp.nanosec * 1e-9:
                self.tf_buffer.popleft()
            # If first tag has older timestamp than tf, remove tag
            else:
                self.tag_buffer.popleft()

    def cleanup(self):
        self.csv_file.close()
        super().destroy_node()

def main():

    parser = argparse.ArgumentParser(description="Record odom at lowered frequency, as well as the Vicon topic.")
    parser.add_argument("--freq", type=float, default=5.0, help="Frequency rate to record messages (Hz)")
    parser.add_argument("--output", type=str, default='test1', help="output csv to record odom topic")
    args = parser.parse_args()

    freq = args.freq
    output = args.output

    rclpy.init()
    node = ImageSubscriber(freq, output)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
        print("\nRecording stopped. CSV saved.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

