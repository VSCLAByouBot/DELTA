#include "delta/Protection.h"

// The Following GearRatio number might be wrong!
// const double GearRatio[AXIS] = {(2835.0 / 32.0), 121.0, 81.0,
//								(2091.0 / 31.0), (2044.0 / 30.0), (2044.0 / 53.0)}; // 減速比
// Correct GearRatio numbers are defined in "Setting.h"
const double GearRatio[AXIS] = {GR_0, GR_1, GR_2, // 減速比
								GR_3, GR_4, GR_5};
std::array<double, AXIS> _POSMIN = {MIN_POS_0, MIN_POS_1, MIN_POS_2, // 原始位置下限
									MIN_POS_3, MIN_POS_4, MIN_POS_5};
std::array<double, AXIS> _POSMAX = {MAX_POS_0, MAX_POS_1, MAX_POS_2, // 原始位置上限
									MAX_POS_3, MAX_POS_4, MAX_POS_5};
std::array<double, AXIS> _VELMAX = {MAX_VEL_0, MAX_VEL_1, MAX_VEL_2, // 原始速度上限
									MAX_VEL_3, MAX_VEL_4, MAX_VEL_5};

// 若無執行 Init_Joint_Bound(),則沿用原始上下限
std::array<double, AXIS> PosMin(_POSMIN), PosMax(_POSMAX), VelMax(_VELMAX);

void Init_Joint_Bound(const double Pos_SF[AXIS], const double Vel_SF[AXIS],
					  const bool checkPos, const bool checkVel,
					  const enum class Unit unit)
{
	for (int i = 0; i < AXIS; ++i)
	{
		PosMin[i] = _POSMIN[i] * Pos_SF[i];
		PosMax[i] = _POSMAX[i] * Pos_SF[i];
		VelMax[i] = _VELMAX[i] * Vel_SF[i];
	}

	std::string unit_str;
	switch (unit)
	{
	case Unit::deg:
		printf("%4s%2s%18s%6s%18s%6s%18s\n", "axis", "|", "PosMin [deg]", "|", "PosMax [deg]", "|", "VelMax [deg/s]");
		printf("%4s%2s%20s%4s%20s%4s%20s\n", "num", "|", "  safe  /   limit", "|", "  safe  /   limit", "|", "  safe  /   limit");
		for (int i = 0; i < AXIS; ++i)
		{
			printf(" J%d  |  %+8.3f / %+8.3f  |  %+8.3f / %+ 8.3f  |  %+8.3f / %+8.3f\n", i + 1,
					rad2deg(PosMin[i]), rad2deg(_POSMIN[i]),
					rad2deg(PosMax[i]), rad2deg(_POSMAX[i]),
					rad2deg(VelMax[i]), rad2deg(_VELMAX[i]));
		}
		break;

	default:
		printf("%4s%2s%18s%6s%18s%6s%18s\n", "axis", "|", "PosMin [rad]", "|", "PosMax [rad]", "|", "VelMax [rad/s]");
		printf("%4s%2s%20s%4s%20s%4s%20s\n", "num", "|", "  safe  /   limit", "|", "  safe  /   limit", "|", "  safe  /   limit");
		for (int i = 0; i < AXIS; ++i)
		{
			printf(" J%d  |  %+8.5f / %+8.5f  |  %+8.5f / %+ 8.5f  |  %+8.5f / %+8.5f\n", i + 1,
					PosMin[i], _POSMIN[i], PosMax[i], _POSMAX[i], VelMax[i], _VELMAX[i]);
		}
		break;
	}
	// 顯示保護模式狀態
	printf("Position Protection: %s\n", checkPos ? "ON" : "OFF");
	printf("Velocity Protection: %s\n", checkVel ? "ON" : "OFF");
}

void Protection(const double Vel[AXIS], int &SafetyFlag)
{
	for (int i = 0; i < AXIS; i++)
	{
		double VelBound = ((3000.0 / 9.55) / GearRatio[i]); // 速度限制(rad/s)
		if (fabs(Vel[i]) > VelBound)
			SafetyFlag = 1;
	}
}

int Check_Joint_State(const double Pos[AXIS], const double Vel[AXIS], double &err,
					  const bool checkPos, const bool checkVel)
{
	for (int i = 0; i < AXIS; ++i)
	{
		// Filter out NaN or +-InF
		if (checkPos && !isfinite(Pos[i]))
		{
			err = Pos[i]; // record error
			return i + 1; // 1 - 6
		}
		if (checkVel && !isfinite(Vel[i]))
		{
			err = Vel[i];		   // record error
			return (i + 1) + AXIS; // 7 - 12
		}
		// if (!isfinite(Acc[i]))
		// {
		// 	err = Acc[i];			   // record error
		// 	return (i + 1) + AXIS * 2; // 13 - 18
		// }
		//  Check Pos
		if (checkPos && (Pos[i] < PosMin[i] || Pos[i] > PosMax[i]))
		{
			err = Pos[i];			   // record error
			return (i + 1) + AXIS * 3; // 19 - 24
		}
		// Check Vel
		if (checkVel && fabs(Vel[i]) > VelMax[i])
		{
			err = Vel[i];			   // record error
			return (i + 1) + AXIS * 4; // 25 - 30
		}
		// Check Acc (TODO)
	}
	return 0;
}

const std::string not_finite_msg = " which is not a finite number.";
// const std::string out_of_range = " is out of range. ";
std::string get_joint_errmsg(int e, double err, const enum class Unit unit)
{
	using std::string;
	using std::to_string;
	// printf("Get joint error code = %d\n", e);
	int err_Joint = (e - 1) % AXIS + 1; // 1 - 6
	int j_ind = err_Joint - 1;			// 1 - 6
	string j_str(to_string(err_Joint));
	string err_val_str(to_string(err));
	string err_val_str_deg(to_string(rad2deg(err)));

	if (1 <= e && e <= AXIS) // 1 - 6
	{
		return string("Pos of J" + j_str + ": " + err_val_str + "," + not_finite_msg);
	}
	else if (AXIS * 1 < e && e <= AXIS * 2) // 7 - 12
	{
		return string("Vel of J" + j_str + ": " + err_val_str + "," + not_finite_msg);
	}
	else if (AXIS * 2 < e && e <= AXIS * 3) // 13 - 18
	{
		return string("Acc of J" + j_str + ": " + err_val_str + "," + not_finite_msg);
	}
	else if (AXIS * 3 < e && e <= AXIS * 4) // 19 - 24
	{
		string _posmin_str, _posmax_str, _err_val_str;
		if (unit == Unit::deg)
		{
			_posmin_str = to_string(rad2deg(PosMin[j_ind]));
			_posmax_str = to_string(rad2deg(PosMax[j_ind]));
			_err_val_str = err_val_str_deg;
		}
		else
		{
			_posmin_str = to_string(PosMin[j_ind]);
			_posmax_str = to_string(PosMax[j_ind]);
			_err_val_str = err_val_str;
		}
		return string("Pos of J" + j_str + ": " + _err_val_str + ", " +
					  _posmin_str + " ~ " + _posmax_str + " required.");
	}
	else if (AXIS * 4 < e && e <= AXIS * 5) // 25 - 30
	{
		string _velmax_str, _err_val_str;
		if (unit == Unit::deg)
		{
			_velmax_str = to_string(rad2deg(VelMax[j_ind]));
			_err_val_str = err_val_str_deg;
		}
		else
		{
			_velmax_str = to_string(VelMax[j_ind]);
			_err_val_str = err_val_str;
		}
		return string("Vel of J" + j_str + ": " + err_val_str + ", " +
					  "-" + _velmax_str + " ~ " + _velmax_str + " required.");
	}
	return string("No matched error code.");
}

void print_joint_errmsg(const double Pos[AXIS], const double Vel[AXIS],
                              const enum class Unit unit)
{
	// Check Pos
	for (int i = 0; i < AXIS; ++i)
	{
		if (!isfinite(Pos[i])) // Filter out NaN or +-InF
			printf("%s\n", get_joint_errmsg(i + 1, Pos[i], unit).c_str());
		else if (Pos[i] < PosMin[i] || Pos[i] > PosMax[i]) // Check Value
			printf("%s\n", get_joint_errmsg(i + 1 + AXIS * 3, Pos[i], unit).c_str());
	}

	// Check Vel
	for (int i = 0; i < AXIS; ++i)
	{
		if (!isfinite(Vel[i])) // Filter out NaN or +-InF
			printf("%s\n", get_joint_errmsg((i + 1) + AXIS, Vel[i], unit).c_str());
		else if (fabs(Vel[i]) > VelMax[i]) // Check Value
			printf("%s\n", get_joint_errmsg(i + 1 + AXIS * 4, Vel[i], unit).c_str());
	}
}

std::string get_joint_state_one_line(const double Pos[AXIS], const double Vel[AXIS],
                                     const enum class Unit unit)
{
	char buffer[200];

	if (unit == Unit::deg)
	{
		sprintf(buffer,
				"%+7.2f %+7.2f %+7.2f %+7.2f %+7.2f %+7.2f  [deg] | "
				"%+8.2f %+8.2f %+8.2f %+8.2f %+8.2f %+8.2f  [deg/s]",
				rad2deg(Pos[0]), rad2deg(Pos[1]), rad2deg(Pos[2]),
				rad2deg(Pos[3]), rad2deg(Pos[4]), rad2deg(Pos[5]),
				rad2deg(Vel[0]), rad2deg(Vel[1]), rad2deg(Vel[2]),
				rad2deg(Vel[3]), rad2deg(Vel[4]), rad2deg(Vel[5]));
	}
	else
	{
		sprintf(buffer,
				"%+7.2f %+7.2f %+7.2f %+7.2f %+7.2f %+7.2f  [rad] | "
				"%+8.5f %+8.5f %+8.5f %+8.5f %+8.5f %+8.5f  [rad/s]",
				Pos[0], Pos[1], Pos[2], Pos[3], Pos[4], Pos[5],
				Vel[0], Vel[1], Vel[2], Vel[3], Vel[4], Vel[5]);
	}
	return std::string(buffer);
}

void init_rnd_gen(std::uniform_real_distribution<double> pos_rnd_gen[AXIS],
				  std::uniform_real_distribution<double> vel_rnd_gen[AXIS])
{
	for (int i = 0; i < AXIS; ++i)
	{
		pos_rnd_gen[i].param(std::uniform_real_distribution<double>::param_type(PosMin[i], PosMax[i]));
		vel_rnd_gen[i].param(std::uniform_real_distribution<double>::param_type(-VelMax[i], VelMax[i]));
	}
}
