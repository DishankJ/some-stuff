# some-stuff

1/6
https://canva.link/hu4mdcbsf3951a6

10/6
https://colab.research.google.com/drive/1xpheg5I3pHyYg4afT9vivPFNIbzTZUh5?usp=sharing

25/6
https://claude.ai/share/909330f6-2571-4289-a47f-db7f9ec0a37a

28/6
https://share.gemini.google/6TMvIvPZLC1N


PART 1: GROUND TRUTH VALUES AND MATHEMATICAL ASSUMPTIONS
================================================================================

1. THE PHYSICAL STEREO SETUP (Extrinsics)
- Camera Baseline (B): 2.0 meters (Horizontal distance between Left and Right cameras)
- Rocket Distance (Z): 4.0 meters (Estimated depth to the center of the rocket)
- Inter-Camera Translation (T_1to2): [0.0, 0.0, 0.0] meters (For stationary hover tests)
- Inter-Camera Rotation (R_1to2): Identity Matrix (Cameras are perfectly parallel)

2. CAMERA SENSOR & LENS (Intrinsics)
- Focal Length (f_px): 2500.0 pixels (Telephoto simulation)
- Image Resolution: 1280 x 1024 pixels
- Principal Point (cx, cy): (640.0, 512.0) (Perfectly centered optical axis)
- Lens Distortion: [0, 0, 0, 0] (Zero radial/tangential distortion)

3. THE ROCKET & TARGETS (Rigid Body)
- Circular Target Radius: 0.025 meters (2.5 cm physical radius of white dots)
- Rigid Body Spacing: ~15 cm horizontal spacing between targets

4. ENVIRONMENTAL SIMULATION
- Sensor Noise Level (Sigma): Adjustable (e.g., 25.0 to 40.0) to simulate ISO grain.
