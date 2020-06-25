Header header           # header

bool inside_wire_left   # state if the left wire sensor is inside (true) the wire loop or outside (false).
uint32 intensity_left   # the intensity is a scale between 0 (very far away) and 100 (directly over the wire) of how far the left sensor is away from the wire.

bool inside_wire_right  # state if the right wire sensor is inside (true) the wire loop or outside (false).
uint32 intensity_right  # the intensity is a scale between 0 (very far away) and 100 (directly over the wire) of how far the right sensor is away from the wire.
