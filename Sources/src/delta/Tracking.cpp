#include "delta/Tracking.h"

Track::Track():
	fp(NULL), line_cnt(0ULL), iter_exec(0ULL)
{

}

int Track::TrackingFile_Init(const char *path, double (&FirstPos)[AXIS])
{
	if (Tracking_OpenFile(path) != 0) // ∂}±“≠y∏Ò©R•O¿…
		return TRACK_FILE_OPEN_ERROR;

	line_cnt = C_countLine(fp);
	printf("TrackingFile_Init(): Quick scan line num. = %llu\n", line_cnt);
	// Resize the Tracking Command Vector
	Track_PosCmd.resize(line_cnt, std::vector<double>(AXIS));
	Track_VelCmd.resize(line_cnt, std::vector<double>(AXIS));
	Track_AccCmd.resize(line_cnt, std::vector<double>(AXIS));

	// Check the Whole Tracking File
	double _PosCmd[AXIS], _VelCmd[AXIS], _AccCmd[AXIS];
	size_t _line = 0ULL, j = 0ULL;

	while (feof(fp) == 0)
	{
		++_line;
		int fmt_check = fscanf_s(fp,
								 "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
								 &_PosCmd[0], &_PosCmd[1], &_PosCmd[2], &_PosCmd[3], &_PosCmd[4], &_PosCmd[5],
								 &_VelCmd[0], &_VelCmd[1], &_VelCmd[2], &_VelCmd[3], &_VelCmd[4], &_VelCmd[5],
								 &_AccCmd[0], &_AccCmd[1], &_AccCmd[2], &_AccCmd[3], &_AccCmd[4], &_AccCmd[5]);

		// The number of the input must be AXIS * 3 = 18
		if (fmt_check != AXIS * 3)
		{
			printf("TrackingFile_Init(): Line %I64u, %d number(s) detected, but require %d.\n", _line, fmt_check, AXIS * 3);
			CloseFile();
			return FMT_NUM_ERROR; // Format number error
		}

		double _err;
		int val_check = Check_Joint_State(_PosCmd, _VelCmd, _err);

		// The input command Pos, Vel, Acc required to be in boundary
		if (val_check != 0)
		{
			// printf("TrackingFile_Init(): Line %I64u, %s\n", _line, get_joint_errmsg(val_check, _err).c_str());
			printf("TrackingFile_Init(): input file \"%s\", Line %I64u\n", filename.c_str(), _line);
			print_joint_errmsg(_PosCmd, _VelCmd);
			CloseFile();
			return val_check; // Value Error
		}

		// Initialize First Position Command
		if (_line == 1ULL)
			for (int i = 0; i < AXIS; i++)
				FirstPos[i] = _PosCmd[i];
		
		// Initialize Command Vector
		Track_PosCmd.at(j) = std::vector<double>(_PosCmd, _PosCmd + AXIS);
		Track_VelCmd.at(j) = std::vector<double>(_VelCmd, _VelCmd + AXIS);
		Track_AccCmd.at(j) = std::vector<double>(_AccCmd, _AccCmd + AXIS);
		j++;
	}

	fseek(fp, 0L, SEEK_SET); // File pointer seek to top
	printf("Tracking File check complete, total line number: %llu\n", _line);
	return 0;
}

int Track::Tracking_OpenFile(const char *path)
{
	printf("Opening Tracking File \"%s\"... ", path);
	fp = C_openFile(path, "r");
	if (fp == NULL)
		return TRACK_FILE_OPEN_ERROR;
	filename = std::string(path);
	puts("OK !");
	return 0;
}

void Track::Get_Cmd_old(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS], double (&AccCmd)[AXIS], int(&EndFlag))
{
	// Original ver.
	if(feof(fp) == false && fscanf_s(fp,
		"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
		&PosCmd[0], &PosCmd[1], &PosCmd[2], &PosCmd[3], &PosCmd[4], &PosCmd[5],
		&VelCmd[0], &VelCmd[1], &VelCmd[2], &VelCmd[3], &VelCmd[4], &VelCmd[5],
		&AccCmd[0], &AccCmd[1], &AccCmd[2], &AccCmd[3], &AccCmd[4], &AccCmd[5]) != EOF )
	{
		EndFlag = 0;
	}
	else
	{
		EndFlag = 1;
	}

	// New ver. (Can detect format error)
	// if (feof(fp) == 0)
	// {
	// 	int ret = fscanf_s(fp,
	// 					   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	// 					   &PosCmd[0], &PosCmd[1], &PosCmd[2], &PosCmd[3], &PosCmd[4], &PosCmd[5],
	// 					   &VelCmd[0], &VelCmd[1], &VelCmd[2], &VelCmd[3], &VelCmd[4], &VelCmd[5],
	// 					   &AccCmd[0], &AccCmd[1], &AccCmd[2], &AccCmd[3], &AccCmd[4], &AccCmd[5]);
	// 	EndFlag = (ret == AXIS * 3 ? 0 : 1); // Keep Reading if return the right number of items of the argument list.
	// }
	// else
	// {
	// 	EndFlag = 1; // Next Time Flag
	// }
}

void Track::Get_Cmd(double (&PosCmd)[AXIS], double (&VelCmd)[AXIS], double (&AccCmd)[AXIS], int(&EndFlag))
{
	if (iter_exec == line_cnt)
	{
		EndFlag = 1;
	}
	else
	{
		std::copy(Track_PosCmd[iter_exec].begin(), Track_PosCmd[iter_exec].end(), PosCmd);
		std::copy(Track_VelCmd[iter_exec].begin(), Track_VelCmd[iter_exec].end(), VelCmd);
		std::copy(Track_AccCmd[iter_exec].begin(), Track_AccCmd[iter_exec].end(), AccCmd);
		iter_exec++;
	}
}

void Track::CloseFile()
{
	if (C_closeFile(fp) != 0)
	{
		puts("Track::CloseFile(): Errors detected!");
		return;
	}
	printf("Track::CloseFile(): \"%s\" closed successfully.\n", filename.c_str());
}
