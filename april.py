import cv2
from dt_apriltags import Detector
import numpy

visualization = True
try:
    import cv2
except:
    raise Exception('You need cv2 in order to run the demo. However, you can still use the library without it.')

try:
    from cv2 import imshow
except:
    print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
    print("VIsualization will be disabled.")
    visualization = False

try:
    import yaml
except:
    raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')


LINE_LENGTH = 5
CENTER_COLOR = (0, 255, 0)
CORNER_COLOR = (255, 0, 255)

### Some utility functions to simplify drawing on the camera feed
# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - LINE_LENGTH, center[1]),
                     (center[0] + LINE_LENGTH, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - LINE_LENGTH),
                     (center[0], center[1] + LINE_LENGTH),
                     color,
                     3)
    return image

# plot a little text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)

cap = cv2.VideoCapture(0)
at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

with open('april_param.yaml', 'r') as stream:
    parameters = yaml.safe_load(stream)


    
detector = Detector()

while(1):

    # Take each frame
    _, frame = cap.read()
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    img = grayscale
    cameraMatrix = numpy.array(parameters['sample_test']['K']).reshape((3,3))
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

    tags = at_detector.detect(img, True, camera_params, parameters['sample_test']['tag_size'])
    

    # color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    if tags:
        # print(tags)
        for tag in tags:
            print("tag_id: %s, center: %s" % (tag.tag_id, tag.center))
            frame = plotPoint(frame, tag.center, CENTER_COLOR)
            frame = plotText(frame, tag.center, CENTER_COLOR, tag.tag_id)

            for idx in range(len(tag.corners)):
                cv2.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))


    if visualization:
        cv2.imshow('Detected tags', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()