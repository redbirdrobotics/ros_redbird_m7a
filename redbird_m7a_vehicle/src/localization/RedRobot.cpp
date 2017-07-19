#include <iostream>
#include <vector>

using namespace std;

RedRobot::RedRobot(int num, int col) : ident(num), color(col)
{
	public:
		std::vector<int> unfound ( 0.begin(), sizeof(r[]).end() );

		// Remove all data from instance
        void wipeRobot()
        {
        	found = False;
        	cam = NULL;
        	radius = 0;
        	framesLost = 0;
        	camProps = {0,0};
        	coords = {0,0};
        	vector = {0,0};
        	mcoords = {0,0};
        	return;
        }

        // Make a list of all Robots that are unfound
        void listUnfound(RedRobot r[], int* p_src, int* p_dst)
        {
        	int* p_dst_begin = p_dst;
        	int* p_dst_end = p_dst + sizeof(*p_dst)/sizeof(int);

        	for (int i = 0; i <= (sizeof(src)/sizeof(int)); i++)
        	{
        		*dst.erase(std::remove(unfoundBegin, unfoundEnd, src[i]), unfoundEnd)
        	}
        }

        // Make a list of 

	private:
	    bool found = False;
	    int cam = NULL;
	    int radius = 0;
	    int framesLost = 0;
	    int camProps[2] = {0,0};
	    int coords[2] = {0,0};
	    int vector[2] = {0,0};
	    int mcoords[2] = {0,0};

}