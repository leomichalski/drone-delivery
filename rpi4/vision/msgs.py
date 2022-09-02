# class Prediction(object):
#     def __init__(self, prediction, probability, label, elapsed_time, image_creation_time):
#         self.prediction = prediction
#         self.probability = probability
#         self.label = label
#         self.elapsed_time = elapsed_time
#         self.image_creation_time = image_creation_time

class ArucoDetection(object):
    def __init__(self, corner_list, id_list, elapsed_time, image_creation_time):
        self.corner_list = corner_list
        self.id_list = id_list
        self.elapsed_time = elapsed_time
        self.image_creation_time = image_creation_time


class Image(object):
    def __init__(self, image, creation_time):
        self.image = image
        self.creation_time = creation_time
