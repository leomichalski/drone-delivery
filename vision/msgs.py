# class Prediction(object):
#     def __init__(self, prediction, probability, label, elapsed_time, image_creation_time):
#         self.prediction = prediction
#         self.probability = probability
#         self.label = label
#         self.elapsed_time = elapsed_time
#         self.image_creation_time = image_creation_time


class ArucoDetection(object):
    def __init__(self, marker_id, marker_center, elapsed_time, image_creation_time):
        self.marker_id = marker_id
        self.marker_center = marker_center
        self.elapsed_time = elapsed_time
        self.image_creation_time = image_creation_time




class Image(object):
    def __init__(self, image, creation_time):
        self.image = image
        self.creation_time = creation_time


class Boolean(object):
    def __init__(self, boolean):
        self.boolean = boolean
