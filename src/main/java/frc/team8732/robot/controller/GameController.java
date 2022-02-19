package frc.team8732.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Contains functions for use with MOST controller (XBox, Playstation,Logitech).
 * @author Joshua Lewis joshlew@trinityforce.org
 * @author bselle
 */
public class GameController extends Joystick {

	private static final double DEADZONE = 0.05;//.1
	private static final double TRIGGER_TOLERANCE = 0.5;

	private final ButtonMap map;

	/**
	 * Constructor that creates a Joystick object.
	 */
	public GameController(int gamepadPort, ButtonMap map) {
		super(gamepadPort);
		this.map = map;
	}

	/**
	 * Returns the X position of the left stick.
	 */
	public double getLeftXAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_LEFT_X));
	}

	/**
	 * Returns the X position of the right stick.
	 */
	public double getRightXAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_RIGHT_X));
	}

	/**
	 * Returns the Y position of the left stick.
	 */
	public double getLeftYAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_LEFT_Y));
	}

	/**
	 * Returns the Y position of the right stick.
	 */
	public double getRightYAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_RIGHT_Y));
	}

	/**
	 * Checks whether Button X/A is being pressed and returns true if it is.
	 */
	public boolean getButtonStateA() {
		return getRawButton(map.BUTTON_A);
	}

	/**
	 * Checks whether Button Circle/B is being pressed and returns true if it is.
	 */
	public boolean getButtonStateB() {
		return getRawButton(map.BUTTON_B);
	}

	/**
	 * Checks whether Button Square/X is being pressed and returns true if it is.
	 */
	public boolean getButtonStateX() {
		return getRawButton(map.BUTTON_X);
	}

	/**
	 * Checks whether Button Triangle/Y is being pressed and returns true if it is.
	 */
	public boolean getButtonStateY() {
		return getRawButton(map.BUTTON_Y);
	}
	
	public boolean getButtonStatePad() {
		return getRawButton(map.BUTTON_TOUCHPAD);
	}

	public int getDpadAngle() {
		return this.getPOV();
	}

	/**
	 * Returns an object of Button A.
	 */
	public Button getButtonA() {
		return new JoystickButton(this, map.BUTTON_A);
	}

	/**
	 * Returns an object of Button B.
	 */
	public Button getButtonB() {
		return new JoystickButton(this, map.BUTTON_B);
	}

	/**
	 * Returns an object of Button X.
	 */
	public Button getButtonX() {
		return new JoystickButton(this, map.BUTTON_X);
	}

	/**
	 * Returns an object of Button Y.
	 */
	public Button getButtonY() {
		return new JoystickButton(this, map.BUTTON_Y);
	}

	public JoystickButton getButtonPad() {
		return new JoystickButton(this, map.BUTTON_TOUCHPAD);
	}

	/**
	 * Gets the state of the Start button
	 * @return the state of the Start button
	 */
	public JoystickButton getOptionsButton(){
		return new JoystickButton(this, map.BUTTON_OPTIONS);
	}

	public JoystickButton getShareButton() {
		return new JoystickButton(this, map.BUTTON_SHARE);
	}
	
	public JoystickButton getStartButton() {
		return new JoystickButton(this, map.BUTTON_START);
	}

	public JoystickButton getLeftBumper() {
		return new JoystickButton(this, map.BUTTON_LEFT_BUMPER);
	}

	public Button getRightBumper() {
		return new JoystickButton(this, map.BUTTON_RIGHT_BUMPER);
	}

	public Button getRightTrigger() {
		return new AxisTriggerButton(this, map.AXIS_RIGHT_TRIGGER, TRIGGER_TOLERANCE);
	}

	public Button getLeftTrigger() {
		return new AxisTriggerButton(this, map.AXIS_LEFT_TRIGGER, TRIGGER_TOLERANCE);
	}

	public Button getDPadUp() {
		return new DPadTriggerButton(this, map.DPAD_UP);
	}

	public Button getDPadDown() {
		return new DPadTriggerButton(this, map.DPAD_DOWN);
	}

	public Button getDPadLeft() {
		return new DPadTriggerButton(this, map.DPAD_LEFT);
	}

	public Button getDPadRight() {
		return new DPadTriggerButton(this, map.DPAD_RIGHT);
	}
	/**
	 * Gets the state of the left stick button
	 * @return the state of the left stick button
	 */
	public JoystickButton getLeftJoystickButton() {
		return new JoystickButton(this, map.BUTTON_LEFT_JOYSTICK);
	}

	/**
	 * Gets the state of the right stick button
	 * @return the state of the right stick button
	 */
	public JoystickButton getRightJoystickButton() {
		return new JoystickButton(this, map.BUTTON_RIGHT_JOYSTICK);
	}

	/**
	 * Gets the state of the left trigger
	 * @return the state of the left trigger
	 */
	public JoystickButton getL2() {
		return new JoystickButton(this, map.BUTTON_LEFT_TRIGGER);
	}

	/**
	 * Gets the state of the right trigger
	 * @return the state of the right trigger
	 */
	public JoystickButton getR2() {
		return new JoystickButton(this, map.BUTTON_RIGHT_BUMPER);
	}

	private boolean inDeadZone(double input){
		boolean inDeadZone;
		inDeadZone = Math.abs(input) < DEADZONE;
		return inDeadZone;
	}

	private double getAxisWithDeadZoneCheck(double input){
		if(inDeadZone(input)){
			input = 0.0;       
		}
		return input; 
	}

	public double getTriggerAxis(int axis){         
		return getAxisWithDeadZoneCheck(this.getRawAxis(axis)); 
	}

	private static class AxisTriggerButton extends Button {
		private final GameController m_controller;
		private final int m_axis;
		private final double m_tolerance;

		public AxisTriggerButton(GameController controller, int axis, double tolerance) {
			m_controller = controller;
			m_axis = axis;
			m_tolerance = tolerance;
		}

		public boolean get() {
			return (m_controller.getTriggerAxis(m_axis) > m_tolerance);
		}
	}

	private static class DPadTriggerButton extends Button {

		private final int buttonAngle;
		private final GameController  controller;

		public DPadTriggerButton(GameController controller, int dPadButtonAngle) {
			this.buttonAngle = dPadButtonAngle;
			this.controller = controller;
		}
		
		@Override
		public boolean get() {
			return controller.getDpadAngle() == buttonAngle;
		}
	}
}