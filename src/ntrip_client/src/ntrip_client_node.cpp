/*
    Originally based from:

        NTRIP client for POSIX.
        $Id: ntripclient.c,v 1.51 2009/09/11 09:49:19 stoecker Exp $
        Copyright (C) 2003-2008 by Dirk Stöcker <soft@dstoecker.de>

    Stripped down and modified to emit ROS messages.
*/
/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
    or read http://www.gnu.org/licenses/gpl.txt
*/

#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include "ros/ros.h"
#include "ntrip_client/ByteArray.h"

#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


#define COMPILEDATE " built " __DATE__

/* The string, which is send as agent in HTTP request */
#define AGENTSTRING "NTRIP NtripClientPOSIX"
#define TIME_RESOLUTION 125

#define MAXDATASIZE 1000 /* max number of bytes we can get at once */

/* CVS revision and version */
static char revisionstr[] = "$Revision: 1.51 $";
static char datestr[]     = "$Date: 2009/09/11 09:49:19 $";

static const char encodingTable [64] = {
  'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
  'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
  'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
  'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
static int encode(char *buf, int size, const char *user, const char *pwd) {
  unsigned char inbuf[3];
  char *out = buf;
  int i, sep = 0, fill = 0, bytes = 0;

    while(*user || *pwd) {
        i = 0;
        while(i < 3 && *user) inbuf[i++] = *(user++);
        if(i < 3 && !sep) {
            inbuf[i++] = ':'; ++sep;
        }
        while(i < 3 && *pwd)  inbuf[i++] = *(pwd++);
        while(i < 3) {
            inbuf[i++] = 0; ++fill;
        }
        if(out-buf < size-1) {
          *(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
        }
        if(out-buf < size-1) {
          *(out++) = encodingTable[((inbuf [0] & 0x03) << 4) | ((inbuf [1] & 0xF0) >> 4)];
        }
        if(out-buf < size-1) {
            if(fill == 2) {
                *(out++) = '=';
            } else {
                *(out++) = encodingTable[((inbuf [1] & 0x0F) << 2) | ((inbuf [2] & 0xC0) >> 6)];
            }
        }

        if(out-buf < size-1) {
            if(fill >= 1) {
                *(out++) = '=';
            } else {
                *(out++) = encodingTable[inbuf [2] & 0x3F];
            }
        }

        bytes += 4;
    }

    if(out-buf < size) {
        *out = 0;
    }

    return bytes;
}

int main(int argc, char **argv) {
    char buf[MAXDATASIZE];
    char req_buf[MAXDATASIZE];
    int req_buf_size;
    struct sockaddr_in their_addr; /* connector's address information */
    struct hostent *he;
    struct servent *se;
    struct timeval socket_timeout = {.tv_sec = 5, .tv_usec = 0};
    char *b;
    long i;
    int numbytes = 0;

    int port;
    std::string server, port_string, username, password, mountpoint;

    ros::init(argc, argv, "ntrip_client_node");
    ros::NodeHandle node("~");

    ros::Publisher dataTopic = node.advertise<ntrip_client::ByteArray>("data", 100);

    if(node.getParam("server", server) == true) {
        ROS_INFO("Server: %s", server.c_str());
    } else {
        ROS_FATAL("Could not get param 'server'");
        return -1;
    }

    if(node.getParam("port", port) == true) {
        std::stringstream temp;
        temp << port;
        port_string = temp.str();
        ROS_INFO("Port: %s", port_string.c_str());
    } else {
        ROS_FATAL("Could not get param 'port'");
        return -1;
    }

    if(node.getParam("mountpoint", mountpoint) == true) {
        ROS_INFO("Mountpoint: %s", mountpoint.c_str());
    } else {
        ROS_FATAL("Could not get param 'mountpoint'");
        return -1;
    }

    if(node.getParam("username", username) == true) {
        ROS_INFO("Username: %s", username.c_str());
    } else {
        ROS_FATAL("Could not get param 'username'");
        return -1;
    }

    if(node.getParam("password", password) == true) {
        ROS_INFO("Password: *******");
    } else {
        ROS_FATAL("Could not get param 'password'");
        return -1;
    }

    memset(&their_addr, 0, sizeof(struct sockaddr_in));
    if((i = strtol(port_string.c_str(), &b, 10)) && (!b || !*b)) {
        their_addr.sin_port = htons(i);
    } else if(!(se = getservbyname(port_string.c_str(), 0))) {
        ROS_ERROR("Can't resolve port %s.", port_string.c_str());
        return -1;
    } else {
        their_addr.sin_port = se->s_port;
    }

    if(!(he=gethostbyname(server.c_str()))) {
        ROS_ERROR("Server name lookup failed for '%s'.", server.c_str());
        return -1;
    }

    their_addr.sin_family = AF_INET;
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);

    // Setup the request header.
    req_buf_size = snprintf(req_buf, MAXDATASIZE-40, /* leave some space for login */
        "GET /%s HTTP/1.1\r\n"
        "Host: %s\r\n%s"
        "User-Agent: %s/%s\r\n"
        "Connection: close%s",
        mountpoint.c_str(),
        server.c_str(),
        "Ntrip-Version: Ntrip/2.0\r\n",
        AGENTSTRING,
        revisionstr,
        (*username.c_str() || *password.c_str()) ? "\r\nAuthorization: Basic " : "");

    req_buf_size += encode(req_buf+i, MAXDATASIZE-i-4, username.c_str(), password.c_str());
    req_buf[req_buf_size++] = '\r';
    req_buf[req_buf_size++] = '\n';
    req_buf[req_buf_size++] = '\r';
    req_buf[req_buf_size++] = '\n';

    ros::Rate retry_rate(0.5f);    // Retry connecting every 2 seconds.

    ROS_INFO("Start NTRIP client.");
    while(ros::ok()){
        int k = 0;
        int chunkymode = 0;
        int chunksize = 0;
        int error = 0;

        int sockfd = 0;

        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            ROS_DEBUG("Failed to open socket.");
            error = 1;
        } else if(setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&socket_timeout,sizeof(struct timeval)) != 0) {
            ROS_DEBUG("Failed to set timeout for socket.");
            error = 1;
        } else if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
            ROS_DEBUG("Failed to connect socket.");
            error = 1;
        } else if(send(sockfd, req_buf, (size_t)req_buf_size, 0) != req_buf_size) {
            ROS_DEBUG("Failed to send.");
            error = 1;
        }

        if(!error){
            ROS_INFO("Successfully connected!");
        }
        while(ros::ok() && !error && (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0) {
            if(!k) {
                buf[numbytes] = 0; /* latest end mark for strstr */

                if((numbytes > 17) &&
                   !strstr(buf, "ICY 200 OK")  &&  /* case 'proxy & ntrip 1.0 caster' */
                   (!strncmp(buf, "HTTP/1.1 200 OK\r\n", 17) ||
                   !strncmp(buf, "HTTP/1.0 200 OK\r\n", 17))) {
                    const char *datacheck = "Content-Type: gnss/data\r\n";
                    const char *chunkycheck = "Transfer-Encoding: chunked\r\n";
                    int l = strlen(datacheck)-1;
                    int j=0;
                    for(i = 0; j != l && i < numbytes-l; ++i) {
                        for(j = 0; j < l && buf[i+j] == datacheck[j]; ++j);
                    }

                    if(i == numbytes-l) {
                        ROS_ERROR("No 'Content-Type: gnss/data' found");
                        error = 1;
                    }

                    l = strlen(chunkycheck)-1;
                    j=0;
                    for(i = 0; j != l && i < numbytes-l; ++i) {
                        for(j = 0; j < l && buf[i+j] == chunkycheck[j]; ++j);
                    }
                    if(i < numbytes-l) {
                        chunkymode = 1;
                    }

                } else if(!strstr(buf, "ICY 200 OK")) {
                    ROS_ERROR("Could not get the requested data.");
                    error = 1;

                } else {
                    ROS_ERROR("NTRIP version 2 HTTP connection failed, falling back to NTRIP1.");
                }

                k = 1;
                char *ep = strstr(buf, "\r\n\r\n");
                if(!ep || ep+4 == buf+numbytes) {
                    continue;
                }
                ep += 4;
                memmove(buf, ep, numbytes-(ep-buf));
                numbytes -= (ep-buf);
            }

            int cstop = 0;
            int pos = 0;
            ntrip_client::ByteArray msg;
            while(!cstop && !error && pos < numbytes) {
                switch(chunkymode) {
                case 1: /* reading number starts */
                    chunksize = 0;
                    ++chunkymode; /* no break */
                case 2: /* during reading number */
                    i = buf[pos++];
                    if(i >= '0' && i <= '9') {
                        chunksize = chunksize*16+i-'0';
                    } else if(i >= 'a' && i <= 'f') {
                        chunksize = chunksize*16+i-'a'+10;
                    } else if(i >= 'A' && i <= 'F') {
                        chunksize = chunksize*16+i-'A'+10;
                    } else if(i == '\r') {
                        ++chunkymode;
                    } else if(i == ';') {
                        chunkymode = 5;
                    } else {
                        cstop = 1;
                    }
                    break;
                case 3: /* scanning for return */
                    if(buf[pos++] == '\n') {
                        chunkymode = chunksize ? 4 : 1;
                    } else {
                        cstop = 1;
                    }
                    break;
                case 4: /* output data */
                    i = numbytes-pos;
                    if(i > chunksize) {
                        i = chunksize;
                    }

                    msg.header.stamp = ros::Time::now();
                    msg.data.insert(msg.data.end(), &(buf[pos]), &(buf[pos + i]));
                    dataTopic.publish(msg);

                    chunksize -= i;
                    pos += i;
                    if(!chunksize) {
                        chunkymode = 1;
                    }
                    break;
                case 5:
                    if(i == '\r') {
                        chunkymode = 3;
                    }
                  break;
                }
            }

            if(cstop) {
                ROS_ERROR("Error in chunky transfer encoding");
                error = 1;
            }
        }

        if((numbytes <= 0) || (error)) {
            ROS_ERROR("Connection failed.");
        }

        if(sockfd) {
            close(sockfd);
        }

        retry_rate.sleep();
    }

    return 0;
}
