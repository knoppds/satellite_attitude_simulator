Version Info: g++.exe 13.1.0
OS Info: Windows 11

-----------------------------

To run, execute 'main.exe'

Simulator will initialize to default satellite orientation (0, 0, 1)

User will be prompted to enter global cartesian coordinates for a new target point in format "X Y Z" (3 signed double precision numbers, space delimited)

Once user presses enter, satellite simulation will begin

Maneuvers are simulated in realtime, with updates being written to the console live

Current planet the satellite is facing is also dynamically updated as the satellite moves, final planet is when the satellite stops moving (last operation is always optical zoom)

Protections are in place for improper input format, and the origin ("0 0 0") is not considered a valid input point

If any issues occur with input, user will be prompted to try again

Close the program by pressing the 'X' in the top right of the Windows CMD window

See "README_notes_and_methodology.png" for additional details and formula derivations