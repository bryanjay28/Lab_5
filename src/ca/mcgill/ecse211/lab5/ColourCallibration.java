package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class ColourCallibration extends Thread{

	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	private static final double[] RGB_blue = { 0.023333, 0.043156, 0.0590942 };
	private static final double[] RGB_red = { 0.122568, 0.019627, 0.013738 };
	private static final double[] RGB_white = { 0.197392, 0.20392, 0.125490 };
	private static final double[] RGB_yellow = { 0.233333, 0.132352, 0.021470 };

	private static final double[] std_blue = { 0.0183301, 0.0157346, 0.0154033 };
	private static final double[] std_red = { 0.0492499, 0.0093156, 0.0090942 };
	private static final double[] std_yellow = { 0.093333, 0.063156, 0.00990942 };
	private static final double[] std_white = { 0.0953342, 0.089224356, 0.05930942 };
	private colour currentBlock;
	private colour flag;
	
	/*
	 * 0 == blue, 1 == red, 2 == yellow, 3 == white
	 */
	public static enum colour {
		BLUE, RED, YELLOW, WHITE
	}

	public ColourCallibration() {
		lightSensor.setCurrentMode("Red"); // set the sensor floodlight to white
	}
	
	public void run() {
		while (true) {
			colourDetection();
		}
	}
	
	/**
	 * Updates the Display to show block colour
	 * 
	 */
	private void UpdateDisplay() {
		String blockColour = "";
		
		if(currentBlock.equals(colour.RED)) {
			blockColour = "Red";
		} else if(currentBlock.equals(colour.BLUE)) {
			blockColour = "Blue";
		} else if(currentBlock.equals(colour.YELLOW)) {
			blockColour = "Yellow";
		} else if(currentBlock.equals(colour.WHITE)) {
			blockColour = "White";
		} else {
			blockColour = null;
		}
		
        LCD.drawString("Block Colour =" + blockColour, 0,5);

	}
	
	/**
	 * Determines if it the target block
	 * 
	 * @return boolean
	 */
	private boolean isBlock() {
		
		if(flag.equals(currentBlock)) {
			Sound.beep();
			return true;
		} else {
			return false;
		}		
	}

	/**
	 * Determines the colour of the block
	 * 
	 */
	private void colourDetection() {

		float[] RGB = new float[3];
		RGB = getRGB();

		if (Math.abs(RGB[0] - RGB_blue[0]) <= 2 * std_blue[0] && Math.abs(RGB[1] - RGB_blue[1]) <= 2 * std_blue[1]
				&& Math.abs(RGB[2] - RGB_blue[2]) <= 2 * std_blue[2]) {
			currentBlock = colour.BLUE;

		} else if(Math.abs(RGB[0] - RGB_red[0]) <= 2 * std_red[0] && Math.abs(RGB[1] - RGB_red[1]) <= 2 * std_red[1]
				&& Math.abs(RGB[2] - RGB_red[2]) <= 2 * std_red[2]) {
			currentBlock = colour.RED;
		} else if(Math.abs(RGB[0] - RGB_yellow[0]) <= 2 * std_yellow[0] && Math.abs(RGB[1] - RGB_yellow[1]) <= 2 * std_yellow[1]
				&& Math.abs(RGB[2] - RGB_yellow[2]) <= 2 * std_yellow[2]) {
			currentBlock = colour.YELLOW;
			
		} else if(Math.abs(RGB[0] - RGB_white[0]) <= 2 * std_white[0] && Math.abs(RGB[1] - RGB_white[1]) <= 2 * std_white[1]
				&& Math.abs(RGB[2] - RGB_white[2]) <= 2 * std_white[2]) {
			currentBlock = colour.WHITE;
		}
		UpdateDisplay();
	}

	/**
	 * Get the RGB values from the colour sensor
	 * 
	 * @return RGB
	 */
	public float[] getRGB() {
		lightSensor.setCurrentMode("RGB");
		SampleProvider colorSensor = lightSensor.getRGBMode();
		float[] RGB = new float[colorSensor.sampleSize()];
		colorSensor.fetchSample(RGB, 0);

		return RGB;
	}

}
