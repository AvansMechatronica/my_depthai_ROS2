# Human pose detection and landmarks for one detected person.

string label

# Detection confidence from person detector.
float32 det_score

# Aggregate landmark score (average keypoint confidence).
float32 lm_score

# Bounding box in normalized image coordinates.
vision_msgs/BoundingBox2D bbox

# Landmarks in normalized image coordinates.
geometry_msgs/Pose2D[] landmark

# Center position in normalized image coordinates.
geometry_msgs/Point position

# True if this detection includes spatial coordinates in meters.
bool is_spatial
