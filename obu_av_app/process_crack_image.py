#! /usr/bin/env python3

'''
This file should contain the main classes that process the raw images with cropping windows
'''

'''
Import lists as three groups, in each group the imports should be ordered alphabetically.
group 1: standard library imports
group 2: third-party library imports installed via pip
group 3: local application/library specific imports
'''

class ImageProcessor(Node):
    def __init__(self, shared_queue):
        super().__init__('image_processor')
        self.get_logger().info('Starting ImageProcessor…')

    def process_image(self, image):
        # Implement the image processing logic here
        pass
