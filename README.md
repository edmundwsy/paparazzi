# Group 12 Project Repo

Paparazzi UAS

=============

[![wakatime](https://wakatime.com/badge/user/c52134f7-9f96-4989-9e12-46bd2ee27bd9/project/1b6ca506-98c0-460c-9e9e-3a5e75d80a68.svg)](https://wakatime.com/badge/user/c52134f7-9f96-4989-9e12-46bd2ee27bd9/project/1b6ca506-98c0-460c-9e9e-3a5e75d80a68)


- Vision modules
  - YUV2BGR color space conversion
  - Morphology operations
  - Connected components
  - 3D projection
- Planning modules
  - Potential field planner (publish velocity commands)
  - State machine for navigation { SAFE, PLANNING, EMERGENCY, OUT_OF_BOUND }

## Branches

- `group12`: default branch
- `opencv`: a simple C++ vision modules using opencv functions, modified by Moji Shi.
- `guided_avoider`: orange avoider (baseline avoider) with our opencv vision module, modified by Joseph Sherman and Koen Bessels
- `cv_pf_avoider`: potential field avoider with our opencv vision module.
- `cv_pf_avoider_emergency`: potential field avoider with vision module and emergency stop.
- `cv_pf_avoider_corner`: potential field avoider with vision modules. Slightly modified the behaviour when drone reaching the boundary: drone will fly to diagonal corners when it goes out of bounds.
- `siyuanwu`: potential field avoider, but static waypoints is published.


## Demo

### Our perception modules
![perception_module](https://user-images.githubusercontent.com/44539400/160805085-810c670e-ed3d-4bbf-9ab2-0d8a99726bfa.png)


### Our potential field planner

https://user-images.githubusercontent.com/44539400/160805164-e4e5eca0-2fbb-4a17-b38b-4646d8000e8b.mp4



