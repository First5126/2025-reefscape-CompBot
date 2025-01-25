# EMP 5126 ReefScape 2025 Practice Robot
[![CI](https://github.com/First5126/2025-reefscape-practice/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/First5126/2025-reefscape-practice/actions/workflows/main.yml)

Welcome to the Parkhill South practice robot code for the [2025 ReefScape][1].

# Prerequisites
* Install [LimeLight Java][2] into project.  Review [LimeLight documentation][3] for usage.
* Install [PathPlannerLib][4] by managing vendor dependencies in VSCode and add online using the json referenced in getting started.
* Install [Phoenix6][5] from CTR Electronics for controlling motors

[1]: https://www.firstinspires.org/robotics/frc/game-and-season
[2]: https://github.com/LimelightVision/limelightlib-wpijava
[3]: https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
[4]: https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib
[5]: https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-replay-frc2025-latest.json

# Lessons Learned

* For AprilTagLocalization on the vision consumer you have to use Utils.fpgaToCurrentTime on the timestamp otherwise it wont work 