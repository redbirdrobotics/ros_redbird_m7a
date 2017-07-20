#include <iostream>
#include <vector>
#include <math.h>

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

        // Populate lists with pointers to Robots that have been found/unfound
        void listByFound(RedRobot* rr[], int rrSize, vector<RedRobot*> foundList[], vector<RedRobot*> unfoundList[])
        {
        	int incF = 0;
        	int incU = 0;

        	for (int i = 0; i <= rrSize; i++)
        	{
        		if rr[i]->found == True
        		{
        			foundList.insert(foundList[incF], rr[i]);
        			incF ++;
        		}
        		else
        		{
        			unfoundList.insert(unfoundList[incU].begin(), rr[i]);
        			incU ++;
        		}

        	}
        	return;
        }

        //Convert stored pixel coordinates to meter coordinates
        void CVT2METER(float xQ, float yQ, float h, float xAxis[], float yAxis[])
        {
        	float theta = xAxis[coords[0]];
        	float phi = yaxis[coords[1]];
        	float yDist = height*tan(phi);
        	float xDist = yDist*tan(theta);
        	mcoords[0] = xDist + xQ;
        	mcoords[1] = yDist + yQ;
        	return;
        }

        //Convert pixel coordinates of all found robots to meter coordinates
        void listCVT2METER(float xQ, float yQ, float h, vector<RedRobot*> foundList[], Camera* camList[])
        {
        	if foundList[].empty()
        	{
        		return
        	}

        	for (int i = 0; i <= foundList.size(); i++)
        	{
        		xAxis = camList[foundList[i]->cam]->xAxis;
        		yAxis = camList[foundList[i]->cam]->yAxis;
        		foundList[i]->CVT2METER(xQ, yQ, h, xAxis, yAxis);
        	}
        	return;
        }

        //Increment framesLost, if threshold met, return True
        bool incFramesLost(int maxFrames)
        {
        	framesLost += 1;
        	if (framesLost == maxFrames)
        	{
        		return True;
        	}
        	return False;
        }

        //Convert framesLost to a scalar to be used for size of ROI
        float CVT2SCALAR(int maxFrames, float maxScalar)
        {
        	float scalarVals[6] = {1,3,4,5.5,7,8.5}
        	return scalarVals[framesLost]
        }

        //Compares coordinates of new blobs to Robots that have framesLost > 0
        //If coordinates are within a designated radius set coordinates to missing Robot
        void checkDist(vector<RedRobot*> foundList[], vector<int> data[], int rad)
        {
        	if foundList.empty()
        	{
        		return
        	}

        	foundLen = foundList.size()

        }

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