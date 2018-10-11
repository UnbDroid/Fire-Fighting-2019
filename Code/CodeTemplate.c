#include <huehue.h>

#define DIGO 10
#define LUCAS 12

typedef struct {
	int mechanic;
	int electronics;
	int prog;
} competition_t;

void MyFunction() {
	printf("Coe Rapaziada");
}

int main() {
	const int VANESSA = 4;
	int rebeca = 25;
	int miniBau = 72;
	competition_t fire_fighting;

	fire_fighting.mechanic = DIGO + VANESSA;
	fire_fighting.electronics = rebeca + miniBau;
	fire_fighting.prog = LUCAS;
  
	MyFunction();
  
	return 0;
}
