#cascade Trainer

import cv2

# Paths to positive and negative images
pos_images_path = 'path/to/positive/images'
neg_images_path = 'path/to/negative/images'

# Parameters for training the classifier
pos_images = cv2.imread(pos_images_path, 0)
neg_images = cv2.imread(neg_images_path, 0)
image_size = (100, 100)
num_pos_images = len(pos_images)
num_neg_images = len(neg_images)
num_stages = 10

# Create a classifier object
cascade = cv2.CascadeClassifier()

# Train the classifier
cascade.train(pos_images, neg_images, None, image_size, num_pos_images, num_neg_images, num_stages, cv2.CASCADE_DO_CANNY_PRUNING)

# Save the classifier as an XML file
cascade.save('classifier.xml')
