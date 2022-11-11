#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include <cstring>
#include <iostream>
#include <sstream>

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1

class serial {
  public:
    serial() {
      fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
      if (fd < 0) {
        perror(MODEMDEVICE);
        exit(-1);
      }
      tcgetattr(fd, & oldtio);
      bzero( & newtio, sizeof(newtio));

      newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
      newtio.c_iflag = IGNPAR | ICRNL;
      newtio.c_oflag = 0;
      newtio.c_lflag = ICANON;
      newtio.c_cc[VEOF] = 4;
      newtio.c_cc[VMIN] = 1;
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd, TCSANOW, & newtio);
    }~serial() {
      close(fd);
    }

  std::string sread() {
    res = read(fd, buf, 255);
    buf[res] = 0;
    return buf;
  }
  
  void swrite(const char * input) {
    write(fd, input, std::strlen(input));
  }
  
  private:
    int fd, c, res;
  struct termios oldtio, newtio;
  char buf[255];
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise < visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(30);

  while (ros::ok()) {
    serial s;
    int recieved_data = 0;
    std::string buf = s.sread();
    std::string st1 = "";
    std::string st2 = "";

    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    
    while ((pos = buf.find(delimiter)) != std::string::npos) {
      token = buf.substr(0, pos);
      st1 = token;
      buf.erase(0, pos + delimiter.length());

    }

    st2 = buf;

    std::cout << st1 << std::endl;
    std::cout << st2 << std::endl;
    int x_received = std::stoi(st1);
    int y_received = std::stoi(st2);

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.action = visualization_msgs::Marker::ADD;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    
    points.scale.x = 5;
    points.scale.y = 5;

    points.color.g = 1.0;
    points.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = x_received;
    p.y = y_received;
    p.z = 0;

    points.points.push_back(p);
    marker_pub.publish(points);
    r.sleep();

  }
}
