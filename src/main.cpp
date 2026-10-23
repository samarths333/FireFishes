#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-9, -10});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({19, 20});
pros::Motor intake(15);
pros::MotorGroup lift_mg({11, -12});

constexpr double LEFT_SCALE   = 1.0;    // lower to reduce power to the left group
constexpr double RIGHT_SCALE  = 0.95;   // lower to reduce power to the right group
constexpr int    DEADZONE     = 8;      // stick values below this are ignored (try 5-12)
constexpr double CURVE        = 0.6;    // 0 = linear, 1 = full cubic (softer near center)
constexpr double TURN_SCALE   = 0.7;    // max turn speed multiplier

constexpr double WHEEL_DIAM_IN = 3.25;   // wheel diameter in inches
constexpr double GEAR_RATIO    = 1.0;   // motor turns per wheel turn
constexpr double DEG_PER_INCH  = 360.0 / (3.14159265 * WHEEL_DIAM_IN) * GEAR_RATIO;
constexpr double TRACK_WIDTH_IN = 12.0;  // PLACEHOLDER: distance between left and right wheel centers, measure it

constexpr double LIFT_MIN  = 0;     // bottom (encoder zero, lift must start fully down)
constexpr double LIFT_MAX  = 420;   // measured top limit, degrees
constexpr double SLOW_ZONE = 60;    // start easing this many degrees before a limit
constexpr int    LIFT_MIN_CMD = 25; // minimum command inside the slow zone so it doesn't stall

double shape(int raw) {
	if (std::abs(raw) <= DEADZONE) return 0.0;
	double sign = raw > 0 ? 1.0 : -1.0;
	double x = (std::abs(raw) - DEADZONE) / (127.0 - DEADZONE);
	double y = CURVE * x * x * x + (1.0 - CURVE) * x;
	return sign * y * 127.0;
}

void drive_in(double inches, int velocity = 100) {
	left_mg.tare_position();
	right_mg.tare_position();
	double deg = inches * DEG_PER_INCH;
	left_mg.move_relative(deg, velocity);
	right_mg.move_relative(deg, velocity);
	uint32_t t0 = pros::millis();
	while (std::abs(left_mg.get_position() - deg) > 5 && pros::millis() - t0 < 4000) pros::delay(10);
	pros::delay(100);
}

void turn_deg(double degrees, int velocity = 85) {
	left_mg.tare_position();
	right_mg.tare_position();
	// each side travels an arc of (pi * track width * degrees / 360) inches
	double deg = (3.14159265 * TRACK_WIDTH_IN * degrees / 360.0) * DEG_PER_INCH;
	left_mg.move_relative(deg, velocity);
	right_mg.move_relative(-deg, velocity);
	uint32_t t0 = pros::millis();
	while (std::abs(left_mg.get_position() - deg) > 5 && pros::millis() - t0 < 4000) pros::delay(10);
	pros::delay(100);
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	lift_mg.set_brake_mode_all(pros::MotorBrake::hold);
	lift_mg.tare_position_all();   // lift must be fully down when the program starts
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}



/**
 * Change start position of DR4B
 */

void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	lift_mg.move_absolute(LIFT_MIN, 80); // lower lift to bottom
	drive_in(24);    // placeholder routine: forward 24"
	drive_in(-12);   // back 12"
	turn_deg(90);    // turn 90 degrees
	drive_in(24);    // placeholder routine: forward 24"
	drive_in(-12);   // back 12"
	turn_deg(90);    // turn 90 degrees
	drive_in(24);    // placeholder routine: forward 24"
	drive_in(-12);   // back 12"
	turn_deg(90);    // turn 90 degrees
	drive_in(24);    // placeholder routine: forward 24"
	drive_in(-12);   // back 12"
	turn_deg(90);    // turn 90 degrees
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	bool intake_on = false;
	while (true) {
		double dir  = shape(master.get_analog(ANALOG_LEFT_Y));
		double turn = shape(master.get_analog(ANALOG_RIGHT_X)) * TURN_SCALE;

		double l = std::fmax(-127, std::fmin(127, dir - turn));
		double r = std::fmax(-127, std::fmin(127, dir + turn));
		left_mg.move(l * LEFT_SCALE);
		right_mg.move(r * RIGHT_SCALE);

		// Intake: R1 hold = in, R2 hold = out, L1 toggles run
		//if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) intake_on = !intake_on;
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))      intake.move(127);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move(-127);
		else intake.move(0);

		int cmd = 0;
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))      cmd = 127;
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) cmd = -127;

		// average both lift motors so one slipping/disconnected doesn't fool the limit
		auto lp = lift_mg.get_position_all();
		double pos = (lp[0] + lp[1]) / 2.0;

		if (cmd > 0) {
			if (pos >= LIFT_MAX) cmd = 0;
			else if (pos > LIFT_MAX - SLOW_ZONE)
				cmd = std::fmax(LIFT_MIN_CMD, cmd * (LIFT_MAX - pos) / SLOW_ZONE);
		} else if (cmd < 0) {
			if (pos <= LIFT_MIN) cmd = 0;
			else if (pos < LIFT_MIN + SLOW_ZONE)
				cmd = std::fmin(-LIFT_MIN_CMD, cmd * (pos - LIFT_MIN) / SLOW_ZONE);
		}
		lift_mg.move(cmd);

		pros::delay(20);
	}
}
