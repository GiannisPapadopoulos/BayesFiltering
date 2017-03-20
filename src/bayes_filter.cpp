/* Autonomous Systems : Bayes filter
// For this assignment you are going to implement a bayes filter

- uniformly distibute the belief over all beliefstates
- implement a representation of the world for every beliefstate
- implement the measurement model
- implement the motion model (forward & turn)
*/

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "laser_to_wall/WallScan.h"
#include "std_msgs/Int32.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <boost/lexical_cast.hpp>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <sstream>

//#include <math.h>
//#include "nav_msgs/Odometry.h"


class BayesFilter {
	public:
	// Construst a new BayesFilter object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	BayesFilter(ros::NodeHandle& nh) : rotateStartTime(ros::Time::now()),rotateDuration(1.8f), moveDuration(0.75f)
	{
		// Initialize random time generator
		srand(time(NULL));
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		// Subscribe to the simulated robot's wall scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		wallSub = nh.subscribe("wall_scan", 1, &BayesFilter::commandCallbackWallScan, this);
		actionSub = nh.subscribe("action", 1, &BayesFilter::commandCallbackAction, this);
		markerPub = nh.advertise<visualization_msgs::MarkerArray>("beliefs",1);
        // poseSub = nh.subscribe("base_pose_ground_truth", 1, &BayesFilter::chatterCallback, this);
        movenoise = false;
		measnoise = false;

		/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
		// Initial belief distribution
		for (int i = 0; i < NUM_STATES; i++) {
            beliefStates.push_back((double) 1 / NUM_STATES);
            tempBeliefs.push_back(0.0);
        }
        ROS_INFO("Beliefs initialized");

        calculateMovementTransitionProbs();
        calculateRotationTransitionProbs();
        initializeExpectedMeasurements();
        /*============================================*/
	};

//    void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
//        int state = (int) floor(msg->pose.pose.position.y) + 5;
//        bool facingLeft = msg->pose.pose.orientation.z > 0 ? true : false;
//        if(!facingLeft) {
//            state = NUM_STATES - state - 1;
//        }
//    }

    // Calculates and stores the state transition probabilities for the move forward action
    // Assumptions:
    //              i) If the robot is facing the wall it remains in the same position with probability 1.0
    //              ii) If robot can only move 1 meter forward, it remains in the same state with probability 0.1
    //                  and move 1 meter forward with probability 0.9
    //
    void calculateMovementTransitionProbs() {
        // Initialize all probabilities to 0
        for (int i = 0; i < NUM_STATES; i++) {
            for (int j = 0; j < NUM_STATES; j++) {
                movementTransitionProbabilitiesNoise[i][j] = 0;
                movementTransitionProbabilitiesDeterministic[i][j] = 0;
            }
        }
        for (int i = 0; i < NUM_STATES; i++) {
            if(i < 8 || (i >= 10 && i < 18 )) { // Can move 1 or 2 meters forward
                movementTransitionProbabilitiesNoise[i][i] = 0.1;
                movementTransitionProbabilitiesNoise[i][i+1] = 0.8;
                movementTransitionProbabilitiesNoise[i][i+2] = 0.1;

                movementTransitionProbabilitiesDeterministic[i][i+1] = 1.0;
            }
            else if (i == 8 || i == 18) { // Can only move 1 meter forward
                movementTransitionProbabilitiesNoise[i][i] = 0.1;
                movementTransitionProbabilitiesNoise[i][i+1] = 0.9;

                movementTransitionProbabilitiesDeterministic[i][i+1] = 1.0;
            }
            else if (i == 9 || i == 19) { // Cannot move forward
                movementTransitionProbabilitiesNoise[i][i] = 1.0;

                movementTransitionProbabilitiesDeterministic[i][i] = 1.0;
            }

        }
        ROS_INFO("Movement transition probabilities initialized");
    }

    // Calculates and stores the state transition probabilities for the rotate 180 degrees action
    void calculateRotationTransitionProbs() {
        // Initialize all probabilities to 0
        for (int i = 0; i < NUM_STATES; i++) {
            for (int j = 0; j < NUM_STATES; j++) {
                rotationTransitionProbabilitiesNoise[i][j] = 0;
                rotationTransitionProbabilitiesDeterministic[i][j] = 0;
            }
        }
        for (int i = 0; i < NUM_STATES; i++) {
            rotationTransitionProbabilitiesNoise[i][NUM_STATES - i - 1] = 0.9;
            rotationTransitionProbabilitiesNoise[i][i] = 0.1;

            rotationTransitionProbabilitiesDeterministic[i][NUM_STATES - i - 1] = 1.0;
        }
        ROS_INFO("Rotation transition probabilities initialized");
    }

    void initializeExpectedMeasurements() {
        ROS_INFO("1-1 %d ", expectedMeasurements[1][1]);
        for(int i = 0; i < NUM_STATES; i++) {
            // wall front
            expectedMeasurements[i][0] = i == 9 || i == 19;
            // wall left
            expectedMeasurements[i][1] = !(i == 1 || i == 7 || i == 16);
            // wall right
            expectedMeasurements[i][2] = !(i == 3 || i == 12 || i == 18);
        }
    }

	// Publish visual information to RVIZ of the beliefstates 
	void publishBeliefMarkers()
	{
		visualization_msgs::MarkerArray beliefs;
		
		for (int i = 0; i < NUM_STATES; i++) {
			visualization_msgs::Marker marker;	
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time();
			marker.ns = "beliefs";
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			
			if (i >= 10) 
			{	
				marker.pose.position.x = -0.8;
				marker.pose.position.y =  4.5 - i%10;
			}	
			else 
			{
				marker.pose.position.x = 0.8;
				marker.pose.position.y = -4.5 + i;
			}
			
			marker.pose.position.z = 0.2;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.5;
			marker.scale.y = 1.0;
			marker.scale.z = 1.0;
			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0  * beliefStates[i];
			marker.id = i;
			beliefs.markers.push_back(marker); 

			// Text
			visualization_msgs::Marker marker2;
			marker2.header.frame_id = "/map";
			marker2.header.stamp = ros::Time();
			marker2.ns = "beliefs";
			marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker2.action = visualization_msgs::Marker::ADD;
	
			if (i >= 10) 
			{	
				marker2.pose.position.x = -0.8;
				marker2.pose.position.y =  4.5 - i%10;
			}	
			else 
			{
				marker2.pose.position.x = 0.8;
				marker2.pose.position.y = -4.5 +i;
			}
			
			marker2.pose.position.z = 0.2;
			marker2.pose.orientation.x = 0.0;
			marker2.pose.orientation.y = 0.0;
			marker2.pose.orientation.z = 0.0;
			marker2.pose.orientation.w = 1.0;
			marker2.scale.x = 0.5;
			marker2.scale.y = 1.0;
			marker2.scale.z = 0.15;
			// Set the color -- be sure to set alpha to something non-zero!
			marker2.color.r = 1.0f;
			marker2.color.g = 1.0f;
			marker2.color.b = 1.0f;
			marker2.color.a = 1.0;
			//std::string text = boost::lexical_cast<string>(i);
			std::ostringstream oss;
			oss << i;	
			std::ostringstream oss2;
			oss2 << beliefStates[i];	
			marker2.text = "State: " + oss.str() + "\nBelief:\n" + oss2.str();
			marker2.id = NUM_STATES + i;
			beliefs.markers.push_back(marker2);    
		}
		
		markerPub.publish(beliefs);
	};
	
	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateMove()
	{
		ROS_INFO("########## Update move ########");
        double (*transitionProbabilities)[20] = movementTransitionProbabilitiesNoise;

        // bel(Xt) = SUM p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
        for (int i = 0; i < NUM_STATES; i++)
        {
            double sum = 0;
            for (int j = 0; j < NUM_STATES; j++)
            {
                sum += transitionProbabilities[j][i] * beliefStates[j];
            }
            tempBeliefs[i] = sum;
        }
        // update the predictions
        for (int i = 0; i < NUM_STATES; i++)
        {
            beliefStates[i] = tempBeliefs[i];
        }
        for (int i = 0; i < NUM_STATES; i++)
        {
            ROS_INFO("Belief state [%d] = [%f]",i,beliefStates[i]);
        }
    };
	/*==========================================*/
	
	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateTurn()
	{
		ROS_INFO("########## Update turn ########");
        double (*transitionProbabilities)[20] = rotationTransitionProbabilitiesNoise;

        // bel(Xt) = SUM p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
        for (int i = 0; i < NUM_STATES; i++)
        {
            double sum = 0;
            for (int j = 0; j < NUM_STATES; j++)
            {
                sum += transitionProbabilities[j][i] * beliefStates[j];
            }
            tempBeliefs[i] = sum;
        }
        // update the predictions
        for (int i = 0; i < NUM_STATES; i++)
        {
            beliefStates[i] = tempBeliefs[i];
        }
        for (int i = 0; i < NUM_STATES; i++)
        {
            ROS_INFO("Belief state [%d] = [%f]",i,beliefStates[i]);
        }
	};
	/*==========================================*/


	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
	void updateSensing()
	{
		ROS_INFO("########## Update sensing ########");
        bool sensorMeasurements[3] = {wall_front, wall_left,  wall_right};
        ROS_INFO("measurements %d %d %d", sensorMeasurements[0], sensorMeasurements[1], sensorMeasurements[2]);

        // calculate observation probabilities p(Zt | Xt) for each state

        double observationProbabilities[NUM_STATES];
        for (int i = 0; i < NUM_STATES; i++)
        {
            // the expected measurements in state i
            bool* expected = expectedMeasurements[i];

            // Combined probability for all 3 sensors
            double totalMeasurementProbability = 1.0;
            for (int j = 0; j < 3; j++) // front, left, right
            {
                if (expected[j])     // state has a wall
                {
                    // 0.8 if sensor also senses wall, else 0.2
                    double measurementProbability = sensorMeasurements[j] ? 0.8 : 0.2;
                    totalMeasurementProbability *= measurementProbability;
                }
                else // State has no wall
                {
                    // 0.7 if sensor also senses no wall, else 0.3
                    double measurementProbability = sensorMeasurements[j] ? 0.3 : 0.7;
                    totalMeasurementProbability *= measurementProbability;
                }
            }
            observationProbabilities[i] = totalMeasurementProbability;

        }
        // Inverse of normalization factor
        double oneOverEta = 0;
        for (int i = 0; i < NUM_STATES; i++)
        {
			oneOverEta += observationProbabilities[i] * beliefStates[i];
        }
        // normalization factor
        double eta = 1/oneOverEta;
        for (int i = 0; i < NUM_STATES; i++)
        {
            beliefStates[i] = eta * observationProbabilities[i] * beliefStates[i];
            ROS_INFO("Belief state [%d] = [%f]",i,beliefStates[i]);
        }
    }
	/*==========================================*/

	// Send a velocity command 
	void move(double linearVelMPS, double angularVelRadPS)
	{
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	}
	// Introduce discrete movement noise
	int movementNoise()
	{
		if (movenoise)
		{ 
			int val = rand()%100;
			
			if (val < LOWER_NOISE_THRESHOLD) return 0;
			if (val >= UPPER_NOISE_THRESHOLD) return 2;
		}
		
		return 1;
	}
	// Introduce measurement noise 
	bool measurementNoise(bool measurement)
	{
		if (measnoise)
		{
			int val = rand()%100;
			
			if (measurement)
			{
				if (val >= 80) return !measurement; 
			}
			else
				if (val >= 70) return !measurement;
    	}
		
		return measurement;
	}
	// Process the incoming action message
	void commandCallbackAction(const std_msgs::Int32::ConstPtr& msg)
	{
		int steps = movementNoise();

		if (msg->data == 0)
		{
    	  		for (int i = 0; i<steps; i++)
    	  		{
					moveStartTime = ros::Time::now();

					while (ros::Time::now() - moveStartTime <= moveDuration)
						move(FORWARD_SPEED_MPS,0);

      				ros::Duration(0.2).sleep();
				}

				updateMove();
		}
		else if (msg->data == 1)
		{
			for (int i = 0; i < std::min(steps,1); i++)
			{
				rotateStartTime = ros::Time::now();
				
				while (ros::Time::now() - rotateStartTime <= rotateDuration) 
					move(0,ROTATE_SPEED_RADPS);
			}
			
			updateTurn();
		}
		else if (msg->data == 2) updateSensing();
		else if (msg->data == 3)
		{
			if (!movenoise) movenoise = true;
			else movenoise = false;
	  
			ROS_INFO_STREAM("movementnoise: " << movenoise);
		}
		if (msg->data == 4)
		{
			if (!measnoise) measnoise = true;
			else measnoise = false;
			
			ROS_INFO_STREAM("measurementnoise: " << measnoise);
		}
	
		publishBeliefMarkers();
	}
	// Process the incoming wall scan message
	void commandCallbackWallScan(const laser_to_wall::WallScan::ConstPtr& msg)
	{
        // ROS_INFO("WallScan");
		wall_left  = measurementNoise((bool)msg->wall_left);
		wall_right = measurementNoise((bool)msg->wall_right);
		wall_front = measurementNoise((bool)msg->wall_front);
		obstacle = (bool)msg->wall_front;
	}
	
	protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Publisher markerPub;
	ros::Subscriber wallSub; // Subscriber to the simulated robot's wall scan topic
	ros::Subscriber actionSub; // Subscriber to the action topic
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
	ros::Time moveStartTime; // Start time of the rotation
	ros::Duration moveDuration; // Duration of the move
    ros::Subscriber poseSub;
    std::vector<double> beliefStates;
	std::vector<double> tempBeliefs;
	std::vector<double> itemsEta;

	bool wall_front, wall_left, wall_right, movenoise, measnoise, obstacle;
	const static int NUM_STATES = 20;
	const static double FORWARD_SPEED_MPS = 1.0;
	const static double ROTATE_SPEED_RADPS = M_PI/2;
	const static int UPPER_NOISE_THRESHOLD = 90;
	const static int LOWER_NOISE_THRESHOLD = 10;
	/*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
    // Element [i][j] is the probability of transitioning from state i to state j
    // after executing 'move forward' with movement noise enabled
    double movementTransitionProbabilitiesNoise[NUM_STATES][NUM_STATES];
    // Transition probabilities for the rotate 180 degrees action, with noise
    double rotationTransitionProbabilitiesNoise[NUM_STATES][NUM_STATES];

    // Transition probabilities for the 'move forward' with noise disable
    double movementTransitionProbabilitiesDeterministic[NUM_STATES][NUM_STATES];
    // Transition probabilities for the rotate 180 degrees action, with noise disabled
    double rotationTransitionProbabilitiesDeterministic[NUM_STATES][NUM_STATES];

    // For each state the actual values of {wall_front, wall_left, wall_right}
    bool expectedMeasurements[NUM_STATES][3];

    /*==========================================*/
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bayes_filter"); // Initiate new ROS node named "bayes_filter"
	ros::NodeHandle n;
	BayesFilter filter(n); // Create new filter object
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
//  ros::spin();
	return 0;
};
