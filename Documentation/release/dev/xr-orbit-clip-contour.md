## XRInterface: orbit, clip and iso surface controls

The XRInterface plugin now orbits the view by default: hold a trigger while
aiming at the data and sweep the controller to move around the point aimed at,
push a thumbstick to move toward or away from it. The data itself is not moved.
The right thumbstick also snap turns by 30 degrees.

Two new pages in the XR menu act on the source selected in the XR pipeline
browser, with changes applied immediately:

- "Clip" creates a Clip or Slice filter with an X, Y, Z or surface aligned
  normal, adjusted with position, rotation and plane size sliders. The plane is
  shown in XR and can be dragged with the controller ray.
- "Iso Surface" creates a Contour filter whose value is set with a slider over
  the range of the chosen point array.
