#include "test.h"

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static dwOps_t dwOps = {NULL};

#define TDOA3_RECEIVE_TIMEOUT 10000

int main(){

	dwInit(dwm, &dwOps);
	dwSetReceiveWaitTimeout(dwm, TDOA3_RECEIVE_TIMEOUT);

	return 0;

}