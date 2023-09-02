# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id

# 2D bounding box surrounding the object in pixels
festival_ur_interfaces/BoundingBox2D bbox

# 3D bounding box surrounding the object in meters
festival_ur_interfaces/BoundingBox3D bbox3d

# segmentation mask of the detected object
# it is only the boundary of the segmented object
festival_ur_interfaces/Mask mask

# keypoints for human pose estimation
festival_ur_interfaces/KeyPoint2DArray keypoints

# keypoints for human pose estimation
festival_ur_interfaces/KeyPoint3DArray keypoints3d
