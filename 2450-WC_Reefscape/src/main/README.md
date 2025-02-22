# Title

> Note: Check out **TODO tree**

> Note: Eliminated pose estimator subsystem and split functionality into drivetrain subsystem and vision subsystem

## Priority Checklist

- [x] Set up photonvision (hardware)
- [x] Set up kraken (hardware and software)
- [ ] Test new radio
- [ ] Beam break (hardware and software) (need cable)
- [ ] Set up new radio (hardware)
- [x] Throughbore encoder (hardware and software)
- [ ] Beam break (hardware and software)
- [ ] CANdle LED (hardware and software)
- [x] Set up default autonomous
- [ ] Convert swerve drive motors (not angle motors) to krakens

Week Zero To-do:
- [x] *Test swerve*
- [x] Basic autonomous
- [ ] Fix vision sequence
- [ ] Controller commands
- [x] Add second limit switch to ElevatorMovement

Week Zero Commands:
- [x] Elevator up/down
- [x] Bop algae
- [x] Left-right swerve micro-adjustment drive

Week Zero Subsystems:
- Coral subsystem
  - [ ] Elevator distance sensor
  - [ ] Beam break

---

  Later To-do:
- [ ] Throughbore encoder
- [ ] Distance sensor (vision)
- [ ] Deep climb subsystem/command
- [ ] Mapping april tag data to network tables (needed for controller)
- [ ] Path planning
- [ ] give jake $20


  Done:
- [x] CANdle LED (hardware and software)
- [x] Set up photonvision (hardware)
- [x] Set up kraken (hardware and software)
- [x] Set up new radio (hardware)
- [x] Set up default autonomous
- [x] Convert swerve drive motors (not angle motors) to krakens


## Current checklist

- Autonomous:
  - Create AutoMethodHelpers file (would include methods like autoBackUp())
  - Make sequential command in Main auto that includes movement and coral functionality

- Hardware bring-up:
  - Throughbore Encoder
  - Distance sensor
  - Magnetic limit switches
  - Beam break

- Set up custom controller:
  - Figure out how to have computer respond to buttons
  - Wiring schematic
  - Serial communication from pc to controller
  - Finish designing the housing

- Convert limelight code to photonvision (marked in TODO tree)
- Add camera limitations for competitions

## Backlog

> These are for things we need to do but are waiting on other dependencies (build team)

- Coral subsystem and corresponding command(s)
  - Map motor controllers to buttons for testing
- Deep climb subsystem and corresponding command(s)
  - Map motor controllers to buttons for testing
- LEDS to the custom controller to show cooldown and elevator height
