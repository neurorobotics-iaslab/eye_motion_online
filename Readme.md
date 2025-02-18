This package chek if there is an eye motion online. 
If the motion occurs then they send a trigger that allow the system to repeat the trial.
Otherwise nothing happens.

Nodes:
    - online_eye_node: it works online and uses the eye_calibration to have the position of the eye whent he subject look to the center
    - offline_eye_node: it works offline and check the motion of the eye during the cf.