# preprocessing
class ImageTransform:
    def __init__(self):
        pass


class ImageClassifier:
    def __init__(self):
        pass


class ImagePredictor:
    def __init__(self, class_names, threshold):
        self.image_classifier = ImageClassifier(
            class_names=self.class_names,
            threshold=self.threshold,
        )
        pass
