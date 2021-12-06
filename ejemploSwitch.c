#include <stdio.h>

int main() {

    char grade = 'H';

    switch (grade) {
        case 'A':
            printf("You did great");
            break;
        case 'B':
            printf("You did very good");
            break;
        case 'C':
            printf("You did good");
            break;
        case 'D':
            printf("You did okay");
            break;
        default :
            printf("Invalid grade");
    }
    return 0;

}