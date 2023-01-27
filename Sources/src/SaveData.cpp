#include "SaveData.h"

SaveData::SaveData():
	fp(NULL), line_cnt(0ULL)
{

}

int SaveData::CreateSaveData(std::string path_prefix, bool add_timestamp)
{
	puts("SaveData Creating ... ");
	if (add_timestamp)
		filename = path_prefix + get_current_sys_time() + ".txt";
	else
		filename = path_prefix;

	fp = C_openFile(filename.c_str(), "w");
	if (fp == NULL)
		return 1;

	printf("SaveData \"%s\" Created OK!\n", filename.c_str());
	return 0;
}

void SaveData::save(double a[])
{
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", a[i]);
	fprintf(fp, "\n");
	++line_cnt;
}

void SaveData::save(double a[], double b[])
{
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", a[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f%s", b[i], i == AXIS - 1 ? "" : " \t");
	fprintf(fp, "\n");
	++line_cnt;
}

void SaveData::save(double a[], double b[], double c[])
{
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", a[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", b[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f%s", c[i], i == AXIS - 1 ? "" : " \t");
	fprintf(fp, "\n");
	++line_cnt;
}

void SaveData::save(double a[], double b[], double c[], double d[], double e[])
{
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", a[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", b[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", c[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", d[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f%s", e[i], i == AXIS - 1 ? "" : " \t");
	fprintf(fp, "\n");
	++line_cnt;
}

void SaveData::save(double a[], double b[], double c[], double d[], double e[], double f[])
{
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", a[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", b[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", c[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", d[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f \t", e[i]);
	for (int i = 0; i < AXIS; i++)
		fprintf(fp, "%f%s", f[i], i == AXIS - 1 ? "" : " \t");
	fprintf(fp, "\n");
	++line_cnt;
}

//================ 關閉實驗資料檔 ================
void SaveData::CloseFile()
{
	if (C_closeFile(fp) != 0)
	{
		puts("SaveData::CloseFile(): Errors detected!");
		return;
	}
	printf("SaveData::CloseFile(): \"%s\" (line num.: %llu) saved successfully.\n",
			filename.c_str(), line_cnt);
}

std::string get_current_sys_time()
{
    std::time_t t = std::time(0);
    std::tm *now = std::localtime(&t);
    char sys_time[40];

    sprintf(sys_time, "%4d%02d%02d_%02d%02d%02d",
            now->tm_year + 1900, now->tm_mon + 1,
			now->tm_mday, now->tm_hour,
			now->tm_min, now->tm_sec);
    return std::string(sys_time);
}
