package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;

public class SearchAndLocalize {
	private double lowerLeftX, lowerLeftY;
	private double upperRightX, upperRightY;
	private int targetBlock;
	private Navigation navigation;
	private boolean foundBlock = false;
	private ColourCalibration colourCalib;
	
	public SearchAndLocalize(double llx, double lly, double urx, double ury, int tb, Navigation nav, ColourCalibration cc) {
		this.lowerLeftX = llx;
		this.lowerLeftY = lly;
		this.upperRightX = urx;
		this.upperRightY = ury;
		this.targetBlock = tb;
		
		this.navigation = nav;
		
		this.colourCalib = cc;
		
		fieldTest();
		
	}
	
	private void fieldTest() {
		this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY);
		Sound.beep();
		double currentTargetX = this.lowerLeftX, currentTargetY = this.lowerLeftY;
		while (targetsArentUpperRight(currentTargetX, currentTargetY) && !foundBlock) {
			currentTargetY += 30.48;
			currentTargetX = switchXValue(currentTargetX);
			this.navigation.travelTo(currentTargetX, currentTargetY);
		}
	}
	
	private boolean targetsArentUpperRight(double currX, double currY) {
		return currX == upperRightX && currY == upperRightY;
	}
	
	private double switchXValue(double x) {
		if (x == this.lowerLeftX) {
			return this.upperRightX;
		} else {
			return this.lowerLeftX;
		}
	}
}
