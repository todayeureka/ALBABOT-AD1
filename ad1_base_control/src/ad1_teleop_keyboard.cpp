#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>


// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
keyboard direction key
up key    : increase only linear speed by 0.1ms
down key  : decrease only linear speed by 0.1ms
right key : increase only angular speed by 0.1r/s
left key  : decrease only angular speed by 0.1r/s

space bar : stop

CTRL-C to quit
)";

// Init variables
float speed(0.0); // Linear velocity (m/s)
float turn(0.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
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
  ros::init(argc, argv, "ad1_teleop_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

  // Create Twist message
  geometry_msgs::Twist msg_cmd_vel;

  char key(' ');

  printf("%s", msg);

  printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);

  while(true){

    key = getch();
    if (key == '\033') { // if the first value is esc
        getch(); // skip the [
        switch(getch()) { // the real value
            case 'A':
                // code for arrow up
                speed += 0.1;
                if(speed > 0.5)
                {
                  speed = 0.5;
                }

                 printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                break;
            case 'B':
                speed -= 0.1;
                if(speed < -0.5)
                {
                  speed = -0.5;
                }
                 printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                break;
            case 'C':
                turn += 0.1;
                if(turn > 0.5)
                {
                 turn = 0.5;
                }

                 printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                break;
            case 'D':
                turn -= 0.1;
                if(turn < -0.5)
                {
                  turn = -0.5;
                }

                 printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                break;
        }
    }
    else if(key == 32)
    {
        speed = 0.0;
        turn = 0.0;
         printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
    }
    else if( key == '\x03')
    {
      break;
      printf("\r\n");
    }

    //printf("input: %d \r\n",getch());
    msg_cmd_vel.linear.x=speed;
    msg_cmd_vel.angular.z=turn;
    pub_cmd_vel.publish(msg_cmd_vel);

    ros::spinOnce();
  }

  return 0;
}
