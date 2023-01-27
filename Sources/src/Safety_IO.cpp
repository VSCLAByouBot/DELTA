#include "Safety_IO.h"

FILE *C_openFile(const char *fileName, const char *mode)
{
	// Check input parameters
	if (fileName == NULL)
	{
		fprintf(stderr, "\nC_openFile(): invalid input.\n");
		return NULL;
	}

	FILE *fp = fopen(fileName, mode);
	if (fp == NULL)
		fprintf(stderr, "\nC_openFile(\"%s\"): %s.\n", fileName, strerror(errno));
	return fp;
}

int C_closeFile(FILE *fp)
{
	// Check input parameters
	if (fp == NULL)
	{
		fprintf(stderr, "\nC_closeFile(): invalid input.\n");
		return INVALID_INPUT;
	}

	if (fclose(fp))
	{
		fprintf(stderr, "\nC_closeFile(): %s.\n", strerror(errno));
		return FCLOSE_ERROR;
	}
	return 0;
}

// 回傳檔案的行數,並將fp重設至檔案開頭
size_t C_countLine(FILE *fp)
{
	/* Descriptions:
	 * File_a.txt:
	 * ```
	 * 123 (Trailing newline example)
	 * 
	 * ```
	 * 
	 * File_b.txt:
	 * ```
	 * 123 (No trailing newline example)
	 * ```
	 * 
	 * File_c.txt (extra traling newline, output behavior is not define):
	 * ```
	 * 123 (This case is not considered, but still show the output result for user)
	 * 
	 * 
	 * ```
	 * 			| File_a.txt | File_b.txt | File_c.txt |
	 * Method 1 |     1 (o)  |     1 (o)  |     2 (?)  |
	 * Method 2 |     2 (x)  |     1 (o)  |     3 (?)  |
	 */

	// Method 1
	const int buf_size = 500;
	char buf[buf_size];
	size_t numLine = 0ULL;

	while (fgets(buf, buf_size, fp) != NULL)
	{
		numLine++;
	}
	
	// Method 2
	/* The following method only counts the number of newline character */
	// char ch;
	// size_t numLine = 0ULL;
	// while(!feof(fp))
	// {
	// 	ch = fgetc(fp);
	// 	if(ch == '\n')
	// 		numLine++;
	// }

	fseek(fp, 0L, SEEK_SET); // File pointer seek to top
	return numLine;
}

