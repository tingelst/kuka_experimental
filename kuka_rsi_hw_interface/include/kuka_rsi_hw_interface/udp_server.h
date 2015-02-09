#include <iostream>
#include <string>

// Select includes
#include <sys/time.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define BUFSIZE 1024

class UDPServer
{
public:
  UDPServer(std::string host, unsigned short port) : local_host_(host), local_port_(port), timeout_(false)
  {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
      std::cout << "ERROR opening socket" << std::endl;
    }
    optval = 1;
    setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));
    memset(&serveraddr_, 0, sizeof(serveraddr_));
    serveraddr_.sin_family = AF_INET;
    serveraddr_.sin_addr.s_addr = inet_addr(local_host_.c_str());
    serveraddr_.sin_port = htons(local_port_);
    if (bind(sockfd_, (struct sockaddr *) &serveraddr_, sizeof(serveraddr_)) < 0)
    {
      std::cout << "ERROR on binding socket" << std::endl;
    }
    clientlen_ = sizeof(clientaddr_);
  }

  ~UDPServer()
  {
    close(sockfd_);
  }

  bool set_timeout(int millisecs)
  {
    if (millisecs != 0)
    {
      tv_.tv_sec  = millisecs / 1000;
      tv_.tv_usec = (millisecs % 1000) * 1000;
      timeout_ = true;
      return timeout_;
    }
    else
    {
      return timeout_;
    }
  }

  ssize_t send(std::string& buffer)
  {
    ssize_t bytes = 0;
    bytes = sendto(sockfd_, buffer.c_str(), buffer.size(), 0, (struct sockaddr *) &clientaddr_, clientlen_);
    if (bytes < 0)
    {
      std::cout << "ERROR in sendto" << std::endl;
    }

    return bytes;
  }

  ssize_t recv(std::string& buffer)
  {
    ssize_t bytes = 0;

    if (timeout_)
    {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sockfd_, &read_fds);

      struct timeval tv;
      tv.tv_sec = tv_.tv_sec;
      tv.tv_usec = tv_.tv_usec;

      if (select(sockfd_+1, &read_fds, NULL, NULL, &tv) < 0)
      {
        return 0;
      }

      if (FD_ISSET(sockfd_, &read_fds))
      {
        memset(buffer_, 0, BUFSIZE);
        bytes = recvfrom(sockfd_, buffer_, BUFSIZE, 0, (struct sockaddr *) &clientaddr_, &clientlen_);
        if (bytes < 0)
        {
          std::cout << "ERROR in recvfrom" << std::endl;
        }
      }
      else
      {
        return 0;
      }

    }
    else
    {
      memset(buffer_, 0, BUFSIZE);
      bytes = recvfrom(sockfd_, buffer_, BUFSIZE, 0, (struct sockaddr *) &clientaddr_, &clientlen_);
      if (bytes < 0)
      {
        std::cout << "ERROR in recvfrom" << std::endl;
      }
    }

    buffer = std::string(buffer_);

    return bytes;
  }

private:
  std::string local_host_;
  unsigned short local_port_;
  bool timeout_;
  struct timeval tv_;

  int sockfd_;
  socklen_t clientlen_;
  struct sockaddr_in serveraddr_;
  struct sockaddr_in clientaddr_;
  char buffer_[BUFSIZE];
  int optval;

};

