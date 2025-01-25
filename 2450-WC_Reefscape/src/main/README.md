# Title

> Note: Check out **TODO tree**0

> Note: Eliminated pose estimator subsystem and split functionality into drivetrain subsystem and vision subsystem

## Priority Checklist

- [x] Set up photonvision (hardware)
- [ ] Set up kraken (hardware and software)
- [ ] Set up new radio (hardware)
- [ ] Throughbore encoder (hardware and software)
- [ ] Beam break (hardware and software)
- [ ] CANdle LED (hardware and software)
- [x] Set up default autonomous
- [ ] Convert swerve drive motors (not angle motors) to krakens



## Current checklist

- Autonomous:
  - Create AutoMethodHelpers file (would include methods like autoBackUp())
  - Make sequential command in Main auto that includes movement and coral functionality

- Hardware bring-up:
  - Throughbore Encoder
  - Distance sensor
  - Kraken
  - CANdle LED
  - Magnetic limit switches
  - Beam break
  - Update radio

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