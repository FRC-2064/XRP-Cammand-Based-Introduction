# XRP Command-Based Project File Structure

This document explains the organization of the XRP robot code and where different pieces of functionality are located.

## Main Robot Files

### `Main.java`
Entry point for the robot program. This file does not need to be modified.

### `Robot.java`
Handles the different modes the robot can be in (disabled, autonomous, teleop, test). The Command Scheduler runs here. This file does not need to be modified.

## Configuration Files

### `RobotContainer.java`
This is where the robot is configured. It handles:
- Creating the controller and XRP robot
- Setting up button bindings for controller inputs
- Defining which autonomous routines are available
- Setting the default command that runs during teleop

Edit this file when:
- Adding new button bindings
- Creating new autonomous options
- Changing controller mappings

### `Constants.java`
All constant values are stored here. This includes:
- Motor and sensor port numbers
- PID gains for gyro correction
- Speed multipliers
- Target distances for autonomous
- Line following thresholds

Edit this file when:
- Tuning drive speeds or behavior
- Adjusting line following sensitivity
- Changing autonomous distances
- Modifying servo positions

## Robot Logic

### `XRP.java`
The superstructure that coordinates all subsystems. It contains:
- State machine for drive modes (manual driving, seek and follow)
- Tank drive, heading hold, and slow mode logic
- Line following algorithm
- Servo control methods

Edit this file when:
- Adding new drive modes
- Changing robot behavior
- Adding features that use multiple subsystems

## Subsystems

### `XRPDrivetrain.java`
Controls all driving-related hardware:
- Left and right motors
- Encoders for distance measurement
- Gyroscope for rotation measurement
- Rangefinder for distance detection
- Reflectance sensors for line detection

Provides methods like `tankDrive()`, `stop()`, `resetGyro()`, and sensor getters.

Edit this file when:
- Adding new sensors
- Creating new sensor access methods
- Changing SmartDashboard outputs

### `Servo.java`
Controls the servo motor. Provides:
- Methods to set preset positions (default, position one, position two)
- Method to set custom angles
- Dashboard output for servo angle

Edit this file when:
- Adding new preset positions
- Changing servo angles
- Adding complex servo movements

## Commands

### `DefaultAutoCommand.java`
Autonomous routine that drives forward 20 inches with:
- Speed ramping at the start
- Constant speed in the middle
- Speed ramping at the end
- Gyro correction to drive straight

Edit this file when:
- Changing drive distance
- Adjusting speed profile
- Adding turns or movements

### `LineFollowAutoCommand.java`
Autonomous routine that searches for and follows a line:
- Drives forward until a line is detected
- Follows the line using `xrp.followLine()`
- Runs until manually stopped

Edit this file when:
- Changing search speed
- Adding distance or time limits
- Modifying end conditions

## Exploration Questions

### Basic Questions

Answer these questions by exploring the code:

1. How would you adjust the home position of the servo?
   - Hint: `Constants.java`, servo positions

2. What constant would you change to make slow mode slower?
   - Hint: `DriveConstants` section

3. Where is the code that determines if the robot is too close to a wall?
   - Hint: Find where `SAFETY_DISTANCE_INCHES` is used

4. How would you add a new button binding for the Options button?
   - Hint: Look at existing button bindings in `RobotContainer.java`

5. What would you change to make the line following turn harder?
   - Hint: `LINE_FOLLOW_TURN_GAIN`

6. Where is the logic that makes the robot scan left and right for a line?
   - Hint: `executeSeekAndFollow()` in `XRP.java`

7. How far does the robot scan when looking for a line?
   - Hint: `SCAN_ANGLE_DEGREES`

8. What percentage of the autonomous path is used to ramp up to full speed?
   - Hint: `RAMP_UP_PERCENT`

9. Which subsystem handles the reflectance sensors?
   - Hint: Check which subsystem creates the sensor objects

10. What happens when you press and hold the Square button?
    - Hint: Button bindings in `RobotContainer.java`

### Intermediate Questions

These questions require writing code:

1. How would you add a method to slowly move the servo from one angle to another over time?
   - Hint: Add a method in `Servo.java` that takes a target angle and speed parameter. You'll need to track the current position and increment it each periodic cycle.

2. How would you create an autonomous command that drives forward until the rangefinder detects a wall?
   - Hint: Create a new command in the `commands` folder that extends `Command`. Use `xrp.getDrivetrain().getRangefinderDistance()` in the `isFinished()` method.

3. How would you add a third preset servo position that can be activated with the Triangle button?
   - Hint: Add a new constant in `ServoConstants`, create a new method in `Servo.java`, then modify the Triangle button binding in `RobotContainer.java`.

4. How would you make the robot drive in a square pattern for autonomous?
   - Hint: Create a new command that uses a state machine to track which side of the square you're on. Use `SequentialCommandGroup` or handle states manually.

5. How would you add a feature that uses the rangefinder to maintain a constant distance from a wall while driving?
   - Hint: Add a new drive state in `XRP.java`. Use a PID controller to adjust turn correction based on the rangefinder distance.

6. How would you create a command that makes the servo sweep back and forth continuously?
   - Hint: Create a new command that tracks sweep direction and target angle. Switch directions when limits are reached.

7. How would you add SmartDashboard buttons to manually select which drive state the robot is in?
   - Hint: Use `SmartDashboard.putData()` to add buttons that run InstantCommands to call `xrp.setDriveState()`.

8. How would you make the line following stop after traveling a certain distance?
   - Hint: Modify `LineFollowAutoCommand.java` to track distance using `xrp.getDrivetrain().getAverageDistanceInch()` and return true in `isFinished()` when the target is reached.

9. How would you add a method to XRPDrivetrain that returns true if either reflectance sensor sees a line?
   - Hint: Add a public method called `isLineDetected()` that checks if either sensor value is above the threshold.

10. How would you create an autonomous that drives until it finds a line, then turns 90 degrees?
    - Hint: Use a `SequentialCommandGroup` with your line-finding command followed by a turn command. The turn command would use the gyro to rotate a specific number of degrees.

## Development Tips

- Make one change at a time and test it
- Define numbers as constants instead of hardcoding them
- Add comments when writing new code
- Test in simulation using `./gradlew simulateJava`
- Commit changes to git regularly
