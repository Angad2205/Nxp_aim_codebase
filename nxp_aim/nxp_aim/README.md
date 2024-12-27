# NXP AIM Grand Finale Repository

This repository contains the code we used on the buggy for the NXP Grand Finale competition.

## Packages Overview

### 1. **b3rb_ros_line_follower**
This package contains the Python nodes and a YOLOv5n-INT8 model. It processes inputs from the camera and LiDAR to:
- Generate edge vectors.
- Detect and classify traffic signs.
- Compute speed and turn values for the buggy.

### 2. **synapse_msgs**
This package defines custom message types for communication between the nodes.

For installation setup, visit [NXP AIM Setup Installation Guide](https://nxp.gitbook.io/nxp-aim/installation-of-nxp-aim-setup).
