#include "catch.hpp"

#include <cstdio>
#include <cstring>

#include <errno.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "../src/CliHandler.hpp"
#include "../src/Daemon.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"

/*
  I realize it's bad to create unit tests which depend on the behaviour of more
  than one object. However, it would be so difficult for me to test the
  CliHandler without a server that I otherwise would have ended up writing no
  tests at all. Better _some_ than _none_.
 */

const std::string testHost{"localhost"};
const int testPort{5111};
const auto networkTimeout = std::chrono::milliseconds(100);

struct SocketSender {
  std::string hostName{""};
  int port{0};
  int sockfd{0};
  bool isConnected{false};

  SocketSender(std::string host, int port) : hostName{host}, port{port} {}
  ~SocketSender() { disconnect(); }

  void connect() {
    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* servinfo;
    {
      // localize scope for rv
      int rv = getaddrinfo(hostName.c_str(), std::to_string(port).c_str(),
                           &hints, &servinfo);

      if (rv != 0) {
        freeaddrinfo(servinfo);
        throw std::runtime_error(
            std::string{"Function getaddrinfo() returned error: "} +
            gai_strerror(rv));
      }
    }

    // loop through serverinfo list.
    struct addrinfo* p;

    for (p = servinfo; p != nullptr; p = p->ai_next) {
      if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) ==
          -1) {
        freeaddrinfo(servinfo);
        throw std::runtime_error(
            std::string{"Function socket() returned error: "} +
            strerror(errno));
      }

      if (::connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
        close(sockfd);
        continue;
      }

      break;  // successfully connected
    }

    if (p == nullptr) {
      freeaddrinfo(servinfo);
      throw std::runtime_error(
          std::string{"Function connect() returned error: "} + strerror(errno));
    }

    freeaddrinfo(servinfo);
    isConnected = true;
  }

  int disconnect() { return close(sockfd); }

  int sendString(std::string message) {
    if (isConnected)
      return send(sockfd, message.c_str(), message.length(), 0);
    else
      return -1;
  }

  int testIsConnected() {
    int error = 0;
    socklen_t len = sizeof(error);
    int returnValue = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
    if (returnValue != 0) {  // there was a problem getting the error code.
      fprintf(stderr, "error getting socket error code: %s\n",
              strerror(returnValue));
    }
    if (error != 0) {  // socket has an error code
      fprintf(stderr, "socket error: %s\n", strerror(error));
      isConnected = false;
    } else
      isConnected = true;
    return returnValue;
  }
};

bool isPointObstacle(Obstacle* obs) {
  return dynamic_cast<PointObstacle*>(obs);
}

TEST_CASE(
    "Using the add obstacle ('ao') commandline should cause an appropriate "
    "obstacle to be placed in the ObstacleContainer") {
  ObstacleContainer obstacles;
  TargetContainer targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};
  Daemon command_server(testPort, commandHandler);
  SocketSender fakeClient(testHost, testPort);

  REQUIRE(obstacles.has_obstacles() == false);
  fakeClient.connect();
  fakeClient.sendString("ao PointObstacle 3.5 4.0 2 .1\n");
  std::this_thread::sleep_for(networkTimeout);

  REQUIRE(obstacles.size() == 1);
  REQUIRE(isPointObstacle(obstacles[0].get()));
  REQUIRE(dynamic_cast<PointObstacle*>(obstacles[0].get())->position.x ==
          Approx(3.5));
  REQUIRE(dynamic_cast<PointObstacle*>(obstacles[0].get())->position.y ==
          Approx(4.0));
  REQUIRE(dynamic_cast<PointObstacle*>(obstacles[0].get())->pwr == 2);
  REQUIRE(dynamic_cast<PointObstacle*>(obstacles[0].get())->eps == Approx(0.1));
}

TEST_CASE(
    "Sending the 'ao' command with too few arguments should result in no "
    "changes made to the ObstacleContainer") {
  ObstacleContainer obstacles;
  TargetContainer targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};
  Daemon command_server(testPort, commandHandler);
  SocketSender fakeClient(testHost, testPort);

  REQUIRE(obstacles.has_obstacles() == false);
  fakeClient.connect();
  fakeClient.sendString("ao PointObstacle 3.5 4.0 2\n");
  std::this_thread::sleep_for(networkTimeout);
  REQUIRE(obstacles.has_obstacles() == false);
}

TEST_CASE(
    "Sending the 'ao' command with a bad argument should result in no changes "
    "made to the ObstacleContainer") {
  ObstacleContainer obstacles;
  TargetContainer targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};
  Daemon command_server(testPort, commandHandler);
  SocketSender fakeClient(testHost, testPort);

  REQUIRE(obstacles.has_obstacles() == false);
  fakeClient.connect();
  fakeClient.sendString("ao PointObstacle 3.5 HELLO 2 0.1\n");
  std::this_thread::sleep_for(networkTimeout);
  REQUIRE(obstacles.has_obstacles() == false);
}

TEST_CASE(
    "After an obstacle is added, sending 'clear obstacles' sould empty the "
    "obstacle container") {
  ObstacleContainer obstacles;
  TargetContainer targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};
  Daemon command_server(testPort, commandHandler);
  SocketSender fakeClient(testHost, testPort);

  REQUIRE(obstacles.has_obstacles() == false);
  fakeClient.connect();
  fakeClient.sendString("ao PointObstacle 3.5 4.0 2 .1\n");
  std::this_thread::sleep_for(networkTimeout);
  REQUIRE(obstacles.has_obstacles() == true);

  fakeClient.sendString("clear obstacles");
  std::this_thread::sleep_for(networkTimeout);
  REQUIRE(obstacles.has_obstacles() == false);
}
