import numpy as np
import os
import tensorflow as tf
import rospy

from styx_msgs.msg import TrafficLight

MIN_DETECTION_THRESHOLD = 0.6
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        current_path = os.path.dirname(os.path.realpath('tl_detector/'))
        rospy.loginfo(current_path)
        path_to_model = current_path + "/TrafficLightRecognModel/ssd_mobilenet_frozen_inference_graph.pb"

        self.detection_graph = tf.Graph()
        self.classes2TL_colors = {1: TrafficLight.GREEN, 2: TrafficLight.RED, 3: TrafficLight.YELLOW, -1: TrafficLight.UNKNOWN}

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(path_to_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def load_image_into_numpy_array(self, image):
        return np.asarray(image)



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for self.detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                def eval_an_image(image):
                    # the array based representation of the image will be used later in order to prepare the
                    # result image with boxes and labels on it.
                    image_np = self.load_image_into_numpy_array(image)
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)

                    # Actual detection.
                    ret = sess.run(
                      [detection_boxes, detection_scores, detection_classes, num_detections],
                      feed_dict={image_tensor: image_np_expanded})

                    return ret, image_np

                (boxes, scores, classes, num), image_np = eval_an_image(image)
                rospy.loginfo("scores")
                rospy.loginfo(scores)
                # rospy.loginfo("num")
                # rospy.loginfo(num)
                idx_max_score = np.argmax(scores)
                # rospy.loginfo("idx_max_score")
                # rospy.loginfo(idx_max_score)

                if max(scores[idx_max_score]) > MIN_DETECTION_THRESHOLD:
                    if classes[0][idx_max_score] == 1.:
                        rospy.loginfo("VERDE")
                    if classes[0][idx_max_score] == 2.:
                        rospy.loginfo("ROSSO")
                    if classes[0][idx_max_score] == 3.:
                        rospy.loginfo("GIALLO")

                    # rospy.loginfo("classes[idx_max_score]")
                    # rospy.loginfo(int(classes[0][idx_max_score]))
                    return self.classes2TL_colors[int(classes[0][idx_max_score])]
                else:
                    rospy.loginfo("TrafficLight.UNKNOWN")
                    return TrafficLight.UNKNOWN
