# LadderControl

2026-02-16 Fix to throw relay before applying power (if configured)
2026-01-27 Upload of a completely new ladder control sketch: LadrCntl

This version supports Arduino analog and digital ports, photoresisters directly connected for 
control or detection, and extended IO boards based on the MCP20317 chip.

It supports:

..Stall or older momentary switch motors

..Optional power control to the motors

..Optional occupancy detection and interlock

..Photoresister inputs directly conected to the Arduino analog ports

See discussion in
https://groups.io/g/arduini/topic/new_ladder_control_sketch/117518719
