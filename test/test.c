#include <stdio.h>
#include <stdlib.h>

typedef enum
{
STATE_1,
STATE_2
} state_t;

int main()
{
	state_t mystate = STATE_1;
	switch(mystate)
	{
		case STATE_1:
			printf("in state1\n");
		case STATE_2:
			printf("in state2\n");
			break;
		default: 
			printf("in default\n");
	}
	return EXIT_SUCCESS;
}
