#include <stdio.h>
int main() 
    {
        int input;
    
    while (1) {
        scanf("%d", &input);
        if (input == -1) {
            break;
        } 
        else if (input == 0) {
            printf("helloworld\n");
        } 
        else if (input == 1) {
            printf("HELLOWORLD\n");
        }
    }
}