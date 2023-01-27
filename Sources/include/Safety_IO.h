#ifndef SAFETY_IO_H
#define SAFETY_IO_H

#include <iostream>

#define INVALID_INPUT 1
#define FCLOSE_ERROR  2

/**
 * @brief Open file with C-style FILE pointer
 * @param fileName: Current joint position [rad]
 * @param mode: Current joint velocity [rad/s]
 * @return File pointer if success, NULL if failed
 */
FILE *C_openFile(const char *fileName, const char *mode);

/**
 * @brief Close file with C-style FILE pointer
 * @param fp: C-style FILE pointer
 * @return 0 if success, 1 if failed
 */
int C_closeFile(FILE *fp);

/**
 * @brief Count the line number of the file
 * @param fp: C-style FILE pointer
 * @return total line number
 */
size_t C_countLine(FILE *fp);

#endif // SAFETY_IO_H
