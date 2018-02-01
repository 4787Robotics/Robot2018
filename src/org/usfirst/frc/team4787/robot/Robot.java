
package org.usfirst.frc.team4787.robot;

//2017

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Talon;



/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
	// min value in x and y direction for robot to respond to joystick movements
	final double DEADZONEX = 0.01, DEADZONEY = 0.01;
	
    // button configuration
	final int  GEAR_MECH_DOOR_OPEN = 3, GEAR_MECH_DOOR_CLOSE = 5, DISABLE_CLIMBING_MECH = 4;
	
    // 0 = joystick usb port in driver station
    Joystick stick = new Joystick(2);
    
    // number in parenthesis for Talon, Servo, and Spark constructor are PWM ports on roborio
    Talon bLeft = new Talon(1);
    Talon fLeft = new Talon(2);
    Talon bRight= new Talon(3);
    Talon fRight = new Talon(4);
    Servo rightGearDoorServo = new Servo(5);
    Servo leftGearDoorServo = new Servo(6);
    Spark climbingMech = new Spark(7);
    
    // Instantiate the compressor.
    // TODO Will - See the Robot() constructor. We may not even need this...
    Compressor c;
    Solenoid s;
    
    //sets up the compressor setting/status
    boolean enabled = c.enabled();
    boolean pressureSwitch = c.getPressureSwitchValue();
    double current = c.getCompressorCurrent();

    // sets RobotDrive obj to null so that auto code works
    RobotDrive arcadeDrive = null;
    
    // variable to handle initialization of Robot Drive obj
    boolean initRobotDrive;
    
    // String variable to determine auto mode, "L" = left starting position, "M" = middle starting position, "R" = right starting posistion
    String autoMode;
    
    // various variables to control robot mechanisms and drive train
    double y, x, expY, expX, autoPower, autoTime, leftDoorAngleStart, rightDoorAngleStart, climbSpeed;
    
    /**
     * Constructs Robot object with necessary configurations
     */
    public Robot() {
    	initRobotDrive = true;
        autoMode = "M";
        CameraServer.getInstance().startAutomaticCapture();
    	Timer.delay(0.05);
    	climbSpeed = 0;
    	
    	// TODO Will
    	//
    	// !! READ ME !!
    	//
    	// It sounds like we don't _need_ a Compressor defined in code. We only really need it if
    	// we want to read some diagnostic info, or if we want fine grain control over it if battery
    	// drain is a problem.
    	//
    	// https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599707-operating-a-compressor-for-pneumatics
       	//
    	// "Creating any instance of a Solenoid or Double Solenoid object will enable the Compressor control on the 
    	// corresponding PCM. The Compressor object is only needed if you want to have greater control over the 
    	// compressor or query compressor status."
    	//
    	// That seems to imply that you needn't worry about the compressor at all, but you can if you want to.
    	boolean on = true; // an alias for 'true'
    	boolean off = false; // an alias for 'false'
    	
    	c = new Compressor(0);
    	// It sounds like closed loop control automatically maintains the pressure for you, which is handy, yeah?
    	// The alternative being that you manually invoke c.start() or c.stop() to control the compressor. Saves
    	// battery since it's not always working to keep consistent pressure, but sounds like it's not worth the trade off.
    	//
    	// It should be noted that it seems closed loop control mode is on by default, so we shouldn't
    	// even need to call this, right?
    	//
    	c.setClosedLoopControl(on);
       	// 
    	// Based on the docs...
    	//
    	// https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599707-operating-a-compressor-for-pneumatics
    	//
    	// ...it sounds like this would automatically enable the compressor, right? We shouldn't need a Compressor
    	// variable. Seems we only need to fire the solenoid to open/close, right?
    	//
    	// Solenoid solenoid = new Solenoid();
    	// solenoid.set(on);
    }
    
    /**
     * Method for robot to cross baseline in auto, causes robot to drive approximate 130 inches (unsure of precise measurement)
     */
    public void crossBaseLine() {
    	autoPower = 0.4;
		autoTime = 2;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(-autoPower);
		fRight.set(-autoPower);
		Timer.delay(autoTime);
    }
    
    public void drive(double speed, double time, boolean isReverse) {
    	speed = isReverse ? -speed : speed;
    	bLeft.set(speed);
		fLeft.set(speed);
		bRight.set(-speed);
		fRight.set(-speed);
		Timer.delay(time);
    }
    
    /**
     * Method for robot to do a 180 degree turn in auto
     */
    public void turnAround() {
    	autoPower = 0.4;
		autoTime = 0.7;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(autoPower);
		fRight.set(autoPower);
		Timer.delay(autoTime);
    }
    
    /**
     * Method for robot to turn left in auto
     */
    public void turnLeft() {
    	autoPower = 0.4;
		autoTime = 0.35;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(autoPower);
		fRight.set(autoPower);
		Timer.delay(autoTime);
    }
    
    /**
     * Method for robot to turn right in auto
     */
    public void turnRight() {
    	autoPower = 0.4;
		autoTime = 0.35;
		bLeft.set(-autoPower);
		fLeft.set(-autoPower);
		bRight.set(-autoPower);
		fRight.set(-autoPower);
		Timer.delay(autoTime);
    }
    
    public void pneumatic(){
        // TODO Will
    	//sets Compressor/pneumatic so that when it is on it closed the pressure tube and vice versa when it is off
    	
    }
    
    /**
     * AUTONOMOUS
     */
    public void autonomous() {
    	switch(autoMode) {
    		case "L":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		case "M":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		/**
    		 * Case for robot to cross baseline in auto (maybe not tested thoroughly), causes from to drive approximately 80-85 inches (unsure of precise measurement)
    		 */
    		case "M2":
    			this.drive(0.4, 0.95, false);
    			this.drive(0.2, 0.91, false);
    			this.stopMotors();
    			Timer.delay(1.0);
    			this.openGearMechDoor();
    			this.drive(0.1, 1.0, true);
    			this.stopMotors();
    			this.closeGearMechDoor();
    			break;
    		/**
    		 * Method for robot to cross baseline in auto, causes robot to drive approximate 130 inches (unsure of precise measurement)
    		 */
    		case "M3":
    			this.drive(0.4, 2.0, false);
    			this.stopMotors();
    			break;
    		case "R":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		default:
    			this.stopMotors();
    			break;
    	}		
    }

    /**
     * Method to set angles on servos so gear mech doors open
     */
    public void openGearMechDoor() {
    	// post build season code (might need to be adjusted if we switch servos) 
		leftGearDoorServo.setAngle(0); //might have to reverse this with the one above
		rightGearDoorServo.setAngle(180);
		
		// build season code
//		leftGearDoorServo.setAngle(leftGearDoorServo.getAngle() + 90);
//		rightGearDoorServo.setAngle(rightGearDoorServo.getAngle() + 90);
    }
    
    /**
     * Method to set angles on servos so gear mech doors close
     */
    public void closeGearMechDoor() {
    	// post build season code (might need to be adjusted if we switch servos)
		leftGearDoorServo.setAngle(180); //might have to reverse this with the one below
		rightGearDoorServo.setAngle(20);
		
		// build season code
//		leftGearDoorServo.setAngle(leftGearDoorServo.getAngle() - 90);
//		rightGearDoorServo.setAngle(rightGearDoorServo.getAngle() - 90);
    }
    
    /**
     * TELEOPERATED MODE. Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	// code to initialize RobotDrive in tele op mode
    	if(initRobotDrive || arcadeDrive.equals(null)) {
        	arcadeDrive = new RobotDrive(fLeft, bLeft, fRight, bRight);
        	initRobotDrive = false;
        }
    	// teleop code loop
    	while (isOperatorControl() && isEnabled()) {
    		// programs motor controllers to drive with exponential arcade drive (i.e. real values are dampened by exponentiation to make driving smoother)
    		expY = Math.pow(-stick.getY(), 1);
    		expX = Math.pow(-stick.getX(), 1);
    		arcadeDrive.arcadeDrive(expY, expX);
    		
    		// retrieves whether our button configurations are pressed down or not
    		boolean openMechDoor = stick.getRawButton(GEAR_MECH_DOOR_OPEN);
        	boolean closeMechDoor = stick.getRawButton(GEAR_MECH_DOOR_CLOSE);
        	boolean disableClimbingMechMotor = stick.getRawButton(DISABLE_CLIMBING_MECH);
        	
        	// determines whether the POV hat switch is set to correct state to trigger events for climbing
        	boolean accelerateClimbingMech = stick.getPOV() == 0 ? true : false;
        	boolean decceraateClimbingMech = stick.getPOV() == 4 ? true : false;
        	
//        	// if GEAR_MECH_DOOR_OPEN button is pressed, open gear mech door so gear falls out
//        	if (openMechDoor) {
//        		this.openGearMechDoor();
//    		}
//        	
//        	// if GEAR_MECH_DOOR_CLOSE button is pressed, close gear mech door so gears will fall in and stay in place on gear mech
//    		if (closeMechDoor) {
//    			this.closeGearMechDoor();
//    		}
//    		
    		// if the DISABLE_CLIMBING_MECH button is pressed, stop the climbing mech motor
        	if (disableClimbingMechMotor) {
        		climbingMech.set(0);
        	}
        	
        	// if the POV stick (hat switch) is moved to the forward position, accelerate the climbing mech motor
        	if (accelerateClimbingMech) {
				climbSpeed += 0.01;
        		climbingMech.set(climbSpeed); //don't know what to set this to
			}
        	
        	// if the POV stick (hat switch) is moved to the forward position, decelerate the climbing mech motor
        	if (decceraateClimbingMech) {
        		climbSpeed -= 0.01;
        		climbingMech.set(climbSpeed);
        	}
    				
    	    Timer.delay(0.005);		// wait for a motor update time
        }
    }

    
    /**
     * Runs during test mode
     */
    public void test() {
    	System.out.println("test function");
    }
    
    /**
     * Method to stop drive train motors
     */
    public void stopMotors() {
    	bLeft.set(0);
		fLeft.set(0);
		bRight.set(0);
		fRight.set(0);
    }
    
    /**
     * Runs while robot is disabled
     */
    public void disabled()
    {
    	// disables all motors and servos on the robot
    	this.stopMotors();
    	rightGearDoorServo.setDisabled();
    	leftGearDoorServo.setDisabled();
    	climbingMech.set(0);
    	System.out.println("I prefer differently abled you ableist");
    }
}
