# Leap-Aided-Modeling-Animation
The project is an addon for Blender that's allow you to exceed the obstacle of working in a 3D space with 2D input devices. Using the Leap Motion controller as a supporting device, the users can interact with Blender for modify, modelling and animating 3D objects. All the code was build in collaboration with <a href="https://www.linkedin.com/in/lucarinaldi91/">Luca Rinaldi</a>.

The code has been tested with:

- Operating system: Linux Ubuntu 14.04 x86_64
- Leap Motion (model LM-010)
- Blender 2.78b
- Python 3.4
- Leap Motion SDK 2.3.1
- Python Scipy module

For a few videos examples please visit this <a href="https://www.youtube.com/watch?v=nES1hmCJBLU&list=PLjBy4bcSz3fo7oJYm3-9Sqqwvg2LwebQR">LINK</a>

-------------------------------------------------- -----------------------------------------
BASIC INFORMATION

For the operation of the code, insert the folder "leapst" in the directory:

<Blender_folder> / <blender_version> / scripts / startup /

To start the addon, after starting Blender press the space bar and select the "Leap Motion Extension" option.

Three modes for editing: Displacement, Selection and Modification (changing from one modality to the next one by pressing Shift).

Three modes for motion along curve: Follow Object, Frenet Frame, Center of Interest.

User view editing operations:
Press the P key to activate it. 
- The pose of recognition is given by the three fingers of the left hand: thumb, index, middle finger. 
  Move the hand in with that pose to rotate and translate the view.
- Displacement with external structure: create a structure similar to the one in the thesis (requires initial calibration)

Interaction operations in Object Mode:
- Move selected object: right index finger in mod. Displacement
- Rotate selected object: right fist in mod. Displacement
- Object selection: right index finger in mod. Selection
- Scaling of selected object: right index finger in mod. Modification

Interaction operations in Edit Mode:
- Select vertices: right two fingers pose to move the sphere selector mod. Displacement
(increase/decrease the sphere radius moving the open hand forward/backward)
- Move selected vertices: right index finger in mod. Displacement
- Rotation of selected vertices: right fist in mod. Displacement
- Scaling of selected vertices: right index finger in mod. Modification

Curves and NURBS surfaces:
- Enter Drawing mode:
  - Press the D key in mod. Displacement
  - Draw the curve starting from the origin with the right index finger
- Create NURBS curve:
  - Press the D key to end the drawing and obtain the resulting NURBS curve (with panel parameters on the left)
  (Alternative: select a 2D polyline curve and use the panel to convert it to NURBS)
- Extrusion surface creation:
  - Press the E key on the keyboard
  - Move the index finger up and down to adjust the height of the surface
  - Press the E key to confirm the height
  (Alternative: select a NURBS curve and use the panel to create an extrusion surface)
- Rotation surface creation:
  - Execute the gesture "Circle" with the index finger of the left hand (the curve around the X axis)
  (Alternative: select a NURBS curve and use the panel to create a rotation surface)
- Creation of skinning surface:
  - Press the J key on the keyboard to draw each successive curve
  - To finish the drawing and generate the surface, press the J key twice in a row
  (Alternative: select multiple 2D NURBS curves along the Z axis and use the panel to create a skinning surface)
- Swinging surface creation:
  (the curve initially drawn is the trajectory, which must be drawn around the point (-1, -1) intended as origin)
  - Press the K key on the keyboard to draw the profile curve on the XZ plane
  - Press the K key to confirm the profile curve and obtain the swinging surface
  (Alternative: select a NURBS curve on the XY plane around the origin, a NURBS curve on the XZ plane and use the panel to create a swinging surface)

Motion along curve:
- Enter Object Mode:
  - Select the position of 3D cursor (Start point of the curve)
  - Select one or more object 
  - Press ; to start
  - Draw the curve starting from the 3D cursor with the right index finger
  - Personalize your settings from the N panel
  - Press ; to create the PATH and bond the object to it

Copyright Information
------------
This code implementation is only for research or education purposes, and not freely available for commercial use. The intellectual property and code implementation belongs to the University of Bologna. I would ask you to contact me if you intend to seriously use, redistribute or publish anything related to this code or repository.
