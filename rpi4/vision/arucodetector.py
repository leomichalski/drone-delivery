import cv2
import time


class ArucoDetector(object):
    def __init__(self):

        # Load the dictionary that was used to generate the markers.
        # There's different aruco marker dictionaries, this code uses 6x6
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

        # Initialize the detector parameters using default values
        self.detector_parameters = cv2.aruco.DetectorParameters_create()

        self.dont_detect_aruco = False
        self.subscriber_list = []


    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def predict(self, img):
        start_time = time.time()

        # # adjust colors for better recognition
        # img = cv2.LUT(frame, self.lut)

        # Convert to grayscale
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detect aruco tags within the frame
        marker_corner_list, marker_id_list, rejected_candidates = cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.detection_parameters)

        # # check duplicate ids
        # idsl = [pid[0] for pid in ids]
        # if len(ids) - len(set(idsl)):
        #     print('duplicate markers on image {}\nmarker ids: {}'.format(
        #         image_name, sorted(idsl)), file=sys.stderr)


        # # # draw box around aruco marker within camera frame
        # # img = cv2.aruco.drawDetectedMarkers(img, marker_corner_list, marker_id_list)

        # # if a tag is found...
        # if marker_id_list is not None:
        #     # for every tag in the array of detected tags...

        #     # flatten the ArUco IDs list
        #     marker_id_list = marker_id_list.flatten()
        #     # loop over the detected ArUCo corners
        #     for (marker_corner, marker_id) in zip(marker_corner_list, marker_id_list):

        #         corner_center = marker_corner[0]
        #         M = cv2.moments(corner_center)
        #         cX = int(M["m10"] / M["m00"])
        #         cY = int(M["m01"] / M["m00"])

        #         # # extract the marker corners (which are always returned in
        #         # # top-left, top-right, bottom-right, and bottom-left order)
        #         # corners = markerCorner.reshape((4, 2))
        #         # (topLeft, topRight, bottomRight, bottomLeft) = corners

        #         # # convert each of the (x, y)-coordinate pairs to integers
        #         # topRight = (int(topRight[0]), int(topRight[1]))
        #         # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        #         # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        #         # topLeft = (int(topLeft[0]), int(topLeft[1]))

        #         # # compute and draw the center (x, y)-coordinates of the ArUco marker
        #         # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        #         # cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        elapsed_time = time.time() - start_time
        return elapsed_time, marker_id_list, marker_corner_list

    def receive_msg(self, msg, topic):
        if topic == topics.TOPIC_DONT_DETECT_ARUCO:
            self.dont_detect_aruco = msg.boolean
            return
        elif topic == topics.TOPIC_IMAGE_ARRAY:
            if self.dont_detect_aruco:
                return
            elapsed_time, marker_id_list, marker_corner_list = self.predict(msg.image_array.copy())
            prediction_msg = msgs.ArucoDetection(corner_list=marker_corner_list, id_list=marker_id_list, elapsed_time=elapsed_time, image_creation_time=msg.creation_time)
            self.publish(prediction_msg, topics.TOPIC_ARUCO_DETECTION)
            return
