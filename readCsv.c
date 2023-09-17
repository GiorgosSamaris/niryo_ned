#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define MAXCHAR 256
char row[MAXCHAR];

int readCSV(FILE* file_ptr, double *data)
{

	fgets(row, MAXCHAR, file_ptr);
    if(feof(file_ptr))   
        return 1;
    
    // printf("Row is: %s", row);
	data[0] = strtod(strtok(row, ","),NULL);
	printf("%lf\n",data[0]);
	for(int i =1; i<6; i++)
	{
		data[i] = strtod(strtok(NULL,","),NULL);
		// printf("%lf\n",data[i]);
	}
	return 0;
}


int main()
{
    double data[6];
    int eof_reached = 0;
    FILE *fp = fopen("solutions.csv","r");
    if (fp == NULL) {
		perror("Failed to open file");
		return 1;
	}
    else{
        printf("File opened successfuly");
	    fgets(row, MAXCHAR, fp);
    }
    while(!readCSV(fp,data))
    {
   
        for(int i =1; i<6; i++)
        {       
            printf("%lf\n",data[i]);
        }
    }
    printf("eof: %d", eof_reached);
    return 0;
}