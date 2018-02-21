package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S3");

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 15.79;
	private static int startingCorner = 0;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("<      | 	   >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Colour|Field   ", 0, 2);
			lcd.drawString(" Sensor|Class   ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			lcd.clear();

			// gotta fix the colour callibration to work independently for the colour test

			ColourCallibration colourCallibration = new ColourCallibration();

			Thread colourCallibrationThread = new Thread(colourCallibration);
			colourCallibration.start();

		} else {

			// Start odometer and display threads and correction Threads.
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// Create ultrasonic and light localizer objects.
			USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance, startingCorner);
			LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor);

			// perform the ultrasonic localization
			USLocalizer.localize();

			// perform the light sensor localization
			lightLocatizer.localize();

			/*
			 * @To Do
			 * 
			 * Use Navigation to navigate to the lower lefthand corner
			 * 
			 * Enter designated since we know LL and Upper Right Search for cube in
			 * designated area (Possible methods snaking the area?) - When you see a block
			 * stop and check its color using Color Calibration - If its not the color use
			 * the avoidance go around it and carry on with the search
			 * 
			 * 
			 */
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}