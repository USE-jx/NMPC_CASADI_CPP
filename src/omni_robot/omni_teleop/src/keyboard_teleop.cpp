#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'k', {0, 0, 0, 0}},
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message
const char* msg = R"(
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

// Init variables
float speed(1.0);              // Linear velocity (m/s)
float turn(1.0);               // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0);  // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    // Create Twist message
    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\rCurrent: speed %f m/s\tturn %f rad/s | Awaiting command...\r", speed, turn);

    while(true){

        // Get the pressed key
        key = getch();

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];

            printf("\rCurrent: speed %f m/s\tturn %f rad/s | Last command: %c   ", speed, turn, key);
        }

        // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            // speed = std::min(speed * speedBindings[key][0], MAX_VEL);
            // turn = std::min(turn * speedBindings[key][1], MAX_ANG);
            float offset = 0.125;
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            printf("\rCurrent: speed %f m/s\tturn %f rad/s | Last command: %c   ", speed, turn, key);
            speed += offset;
        }

        // Otherwise, set the robot to stop
        else
        {
            x = 0;
            y = 0;
            z = 0;
            th = 0;

            // If ctrl-C (^C) was pressed, terminate the program
            if (key == '\x03') break;

        printf("\rCurrent: speed %f m/s\tturn %f rad/s| Invalid command! %c", speed, turn, key);
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);

        ros::spinOnce();
    }

    return 0;
}