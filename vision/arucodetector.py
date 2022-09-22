import time

import cv2
import numpy as np

import topics
import msgs


class ArucoDetector(object):
    def __init__(self):

        # Load the dictionary that was used to generate the markers.
        # There's different aruco marker dictionaries, this code uses 5x5
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)

        # Initialize the detector parameters using default values
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.perspectiveRemoveIgnoredMarginPerCell = 0.3  # default 0.13
        self.detector_parameters.minMarkerPerimeterRate = 0.01  # default 0.03

        self.lookup_table = np.interp(np.arange(0, 256), [0, 158, 216, 255], [0, 22, 80, 176]).astype(np.uint8)

        self.dont_detect_aruco = False
        self.subscriber_list = []


    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def predict(self, img):
        start_time = time.time()

        img = cv2.LUT(img, self.lookup_table)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        marker_corner_list, marker_id_list, rejected_candidates = cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.detector_parameters)

        # # # draw box around aruco marker within camera frame
        # # img = cv2.aruco.drawDetectedMarkers(img, marker_corner_list, marker_id_list)
        marker_center_list = []

        # if a tag is found...
        if marker_id_list is None:
            return 0, [], []

        # for every tag in the array of detected tags...

        # flatten the ArUco IDs list
        marker_id_list = marker_id_list.flatten()
        # TODO: remove duplicate ids from marker_id_list and marker_corner_list

        # loop over the detected ArUCo corners
        for (marker_corner, marker_id) in zip(marker_corner_list, marker_id_list):
            # TODO: soh considerar deteccao se o marker tiver 20x20cm mesmo

            corner_center = marker_corner[0]
            M = cv2.moments(corner_center)
            # cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])
            marker_center_list.append([
                int(M["m10"] / M["m00"]),
                int(M["m01"] / M["m00"])
            ])

            # # extract the marker corners (which are always returned in
            # # top-left, top-right, bottom-right, and bottom-left order)
            # corners = markerCorner.reshape((4, 2))
            # (topLeft, topRight, bottomRight, bottomLeft) = corners

            # # convert each of the (x, y)-coordinate pairs to integers
            # topRight = (int(topRight[0]), int(topRight[1]))
            # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            # topLeft = (int(topLeft[0]), int(topLeft[1]))

            # # compute and draw the center (x, y)-coordinates of the ArUco marker
            # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            # cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        elapsed_time = time.time() - start_time
        # TODO: return aruco marker direction (north, east, south, west, etc)
        return elapsed_time, marker_id_list, marker_center_list

    def receive_msg(self, msg, topic):
        if topic == topics.TOPIC_DONT_DETECT_ARUCO:
            self.dont_detect_aruco = msg.boolean
            return
        elif topic == topics.TOPIC_IMAGE_ARRAY:
            if self.dont_detect_aruco:
                return

            elapsed_time, marker_id_list, marker_center_list = self.predict(msg.image.copy())

            # publish a separate message for each marker id
            # for id, center in zip(marker_id_list, marker_center_list):
            #     prediction_msg = msgs.ArucoDetection(marker_id=id, marker_center=center, elapsed_time=elapsed_time, image_creation_time=msg.creation_time)
            #     self.publish(prediction_msg, topics.TOPIC_ARUCO_DETECTION)
            prediction_msg = msgs.ArucoDetection(marker_id=1, marker_center=[1,2], elapsed_time=31312, image_creation_time=msg.creation_time)
            self.publish(prediction_msg, topics.TOPIC_ARUCO_DETECTION)


            print(
                "ids:", marker_id_list,
                "corners:", marker_center_list,
                "elapsed_time:", elapsed_time,
                # "FPS:", round(1/elapsed_time),
                "image_creation_time:", msg.creation_time,
            )
