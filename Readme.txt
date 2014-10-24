% This program is free software: you can redistribute it and/or modify      
% it under the terms of the GNU General Public License version 2 as         
% published by the Free Software Foundation.                                
%                                                              
% This program is distributed in the hope that it will be useful,           
% but WITHOUT ANY WARRANTY; without even the implied warranty of            
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             
% GNU General Public License for more details.                              
%                                                              
% You should have received a copy of the GNU General Public License         
% along with this program.  If not, see <http://www.gnu.org/licenses/>.  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

This is a path following controller designed for the SIG Rascal 110 aircraft that operates within JSBsim as a part of the APM SITL setup.
The controller uses the L1 Guidance algorithm from: S. Park, J. Deyst, and J. P. How, “A New Nonlinear Guidance Logic for Trajectory Tracking”, Proceedings of the AIAA Guidance, Navigation and Control Conference, Aug 2004. AIAA-2004-4900 (PDF).
Authors: Will Silva, Tevis Nichols, Steve McGuire, Paul Guerrie, Matthew Aitken, Aaron Buysse
WARNING: The aircraft is set to FBWA in this controller.
WARNING: All gains are tuned for this SIG Rascal 110 aircraft and WILL NOT WORK ON ANY OTHER AIRCRAFT WITHOUT MODIFYING GAINS.
WARNING: This has NOT been tested in live flight yet

Usage:

1. Setup SITL on Linux 
http://dev.ardupilot.com/wiki/setting-up-sitl-on-linux/

2. Setup DroneAPI
http://dev.ardupilot.com/wiki/droneapi-tutorial/
Be sure to do the following command as part of the DroneAPI setup (assuming you are in $home):
echo "module load droneapi.module.api" >> ~/.mavinit.scr

3. Install APM Planner 2
http://ardupilot.com/downloads/?category=35

3. Using Terminal, navigate to your ArduPlane directory. Usually in /home/<username>/ardupilot/ArduPlane
cd ~/ardupilot/ArduPlane

4. Clone suas-guidance into ArduPlane directory
git clone https://code.google.com/p/suas-guidance/

5. Overwrite DroneAPI "api.py". Usually in /usr/local/lib/python2.7/droneapi/module/
cd suas-guidance
sudo cp droneAPI/api.py /usr/local/lib/python2.7/dist-packages/droneapi/module/

6. Launch APM Planner 2
apmplanner2

7. Use csv2flightplan.py to generate a flightplan.txt file to load into APM Planner 2 (optional)
   Used for visualization of the flightplan ONLY
   Note 1: Flight plan must be in NED inertial (Y,X,-Z) coordinates in a CSV file. See "potatochip.csv" as example.
   Note 2: Origin for path is currently set in Controller.py
   Note 3: THIS PLAN WILL NOT BE SENT TO THE AIRCRAFT VIA WAYPOINTS

8. Launch SITL
cd ..
sim_vehicle.sh --map --console --aircraft test

9. Launch Flight Gear (optional)
$home/ardupilot/Tools/autotest/$ ./fg_plane_view.sh

10. Ensure your desired flight path csv is in /paths directory
    Note: Origin for path is currently set in Controller.py

11. Once SITL is up and running, start controller in MAVProxy terminal window
api start ./suas-guidance/controller/controller.py

Off you go!




