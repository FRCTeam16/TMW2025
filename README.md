# TMW2025 REEFSCAPE

## 2025 Robot Control Specs
D-pad modifier:
    `Left ` : Algea Elevator Positions
    `Up   ` : Coral Elevator Postitions
    `Right` : Test / Open Loop
    `Down ` : Climber Sequence

    The D-pad controls act as a set of modifier buttons for the `A`, `B`, `X`, & `Y` buttons

    `Left` Algea Elevator Positions:
        - `A` Lower Reef Algea Pose
        - `B` Upper Reef Algea Pose
        - `X` Processor Algea Pose
        - `Y` Barge Algea Pose

    `Up` Coral Elevator Poses:
        - `A` Coral Trough Position
        - `B` Coral Lvl.1 Position
        - `X` Coral Lvl.2 Position
        - `Y` Coral Lvl.3 Position

    `Right` Additional controls ( Test and Open Loop ):
        - tbd

    `down` Climber Final Staging:
        - `B` Advance Climber Stage
        - `X` Reverse Climber Stage
    
Top-of-sticks Controls:
    Every command executable via the top of the sicks will use 


## Robot Config Variants
You can register a new robot variant configuration, see `src/main/java/frc/robot/generated/README.md` for details.

## Tools
### PathPlanner Reef Coords
Use the gradle target `runGenerateReefCoords` to generate the reef coordinates for the path planner.
The output will be in the `src/main/deploy/pathplanner/paths/{color}CoordPath_{timestamp}.path` file
and shown on standard out. 
