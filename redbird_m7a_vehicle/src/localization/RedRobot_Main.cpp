#include <iostream>
#include <vector>
#include "RedRobot.h"


using namespace std;

RedRobot RRa(0,2);
RedRobot RRb(1,2);
RedRobot RRc(2,2);
RedRobot RRd(3,2);
RedRobot RRe(4,2);

RedRobot* RRa_p = &RRa;
RedRobot* RRb_p = &RRb;
RedRobot* RRc_p = &RRc;
RedRobot* RRd_p = &RRd;
RedRobot* RRe_p = &RRe;

RedRobot* RRlist[5] = {RRa_p, RRb_p, RRc_p, RRd_p, RRe_p};
int RRlistSize = RRlist + sizeof(RRlist)/sizeof(RedRobot)

std::vector<RedRobot*> unfoundList;
std::vector<RedRobot*> foundList;
std::vector<int> data;

int main()
{
	
}