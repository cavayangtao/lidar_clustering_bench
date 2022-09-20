# evaluation code

clustering_clean.py is used to clean the output results from adaptive clustering, euclidean clustering, and run clustering when the input point cloud misses "intensity" or "ring".

time_clean.py is used to format the output results of runtime of depth clustering.

iou_timestamp.py is used to calculate 3D IoU when the output results and the ground truth are matched by timestamp.

iou_frame.py is used to calculate 3D IoU when the output results and the ground truth are matched by frame number. (for depth clustering only)

time_plot.py is used to plot the run-time curves of different algorithms.
