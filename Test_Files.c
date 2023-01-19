#include <stdio.h>
#include <curses.h> // -> use this if you are using windows
//#include <conio.h> -> use this if you are using linux
#include <stdlib.h>
#include <string.h>

int main(){
    FILE* fp = fopen("./curva_reflow.csv", "r");
    if (!fp){
        printf("Can't open file\n");
    }
    else {
        // Here we have taken size of
        // array 1024 you can modify it
        char buffer[1024];
 
        int row = 0;
        int column = 0;
 
        while (fgets(buffer,
                     1024, fp)) {
            column = 0;
            row++;
 
            // To avoid printing of column
            // names in file can be changed
            // according to need
            if (row == 1)
                continue;
 
            // Splitting the data
            char* value = strtok(buffer, ", ");
 
            while (value) {
                // Column 1
                if (column == 0) {
                    printf("Name :");
                }
 
                // Column 2
                if (column == 1) {
                    printf("\tAccount No. :");
                }
 
                // Column 3
                if (column == 2) {
                    printf("\tAmount :");
                }
 
                printf("%s", value);
                value = strtok(NULL, ", ");
                column++;
            }
 
            printf("\n");
        }
 
        // Close the file
        fclose(fp);
    }

    return 0;
}