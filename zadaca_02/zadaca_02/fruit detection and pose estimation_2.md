Implement a pipeline that uses your trained 2D instance segmentation model to segment the corresponding regions in a point cloud and estimate the 3D centroid of fruits in a cluttered scene (e.g., bowl/box with overlapping fruits).

For each detected instance:

Map segmented 2D pixels to 3D points (RGB–D alignment).
Extract the instance point cloud.
Estimate centroid using:
geometric model fitting, and
3D bounding box extraction.
Compare both centroid estimates and visualize results.
Each student performs evaluation only on their assigned fruit class.

Due: Wednesday, 29 April 2026, 12:00 AM
