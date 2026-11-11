# P1_DrivingRange

The goal of this project is to have a Zumo32U4 follow a premade path on the ground, made with black tape, scanning for small balls on the range. 
Once a ball is found, using the proximity sensor on the right side, the zumo will divert from the path to go pick up the ball. 
Afterwards it's return to where it left the path and continue along in the same direction as before it detected the ball. 
Once it's reached the end of the path, it'll turn around and repeat scanning the other side of the range.
This way we can ensure the whole driving range is scanned with only one round trip.

## Dependencies
This project uses the Zumo32U4 library by Pololu aswell as the PID library by Brett Beauregard, both of which can be found in the arduino IDE libraries.
