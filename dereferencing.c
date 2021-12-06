#include <stdio.h>
#include <stdlib.h>

int main() {

    int age = 30;
    int *pAge = &age;

    printf("%d", *&age);
    printf("\nModificacion en github");
}