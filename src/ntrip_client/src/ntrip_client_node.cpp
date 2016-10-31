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
#include "std_msgs/ByteMultiArray.h"

#include <signal.h>
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

enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

struct Args {
  const char *server;
  const char *port;
  const char *user;
  const char *password;
  const char *data;
  int         bitrate;
  int         mode;
};


int stop = 0;

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
    struct Args args;

    int error = 0;
    int sockfd = 0;
    int numbytes;
    char buf[MAXDATASIZE];
    struct sockaddr_in their_addr; /* connector's address information */
    struct hostent *he;
    struct servent *se;
    char *b;
    long i;

    std::string server, port, username, password, mountpoint;

    ros::init(argc, argv, "ntrip_client_node");
    ros::NodeHandle node("~");

    std::string data_topic_name = ros::this_node::getName() + "/data";
    ros::Publisher dataTopic = node.advertise<std_msgs::ByteMultiArray>(data_topic_name, 100);

    if(node.getParam("server", server) == true) {
        ROS_INFO("Server: %s", server.c_str());
    } else {
        ROS_ERROR("Could not get param 'server'");
        return -1;
    }

    if(node.getParam("port", port) == true) {
        ROS_INFO("Port: %s", port.c_str());
    } else {
        ROS_ERROR("Could not get param 'port'");
        return -1;
    }

    if(node.getParam("mountpoint", mountpoint) == true) {
        ROS_INFO("Mountpoint: %s", mountpoint.c_str());
    } else {
        ROS_ERROR("Could not get param 'mountpoint'");
        return -1;
    }

    if(node.getParam("username", username) == true) {
        ROS_INFO("Username: %s", username.c_str());
    } else {
        ROS_ERROR("Could not get param 'username'");
        return -1;
    }

    if(node.getParam("password", password) == true) {
        ROS_INFO("Password: *******");
    } else {
        ROS_ERROR("Could not get param 'password'");
        return -1;
    }

    memset(&their_addr, 0, sizeof(struct sockaddr_in));
    if((i = strtol(port.c_str(), &b, 10)) && (!b || !*b)) {
        their_addr.sin_port = htons(i);
    } else if(!(se = getservbyname(port.c_str(), 0))) {
        ROS_ERROR("Can't resolve port %s.", port.c_str());
        return -1;
    } else {
        their_addr.sin_port = se->s_port;
    }

    if(!(he=gethostbyname(server.c_str()))) {
        ROS_ERROR("Server name lookup failed for '%s'.", server.c_str());
        return -1;
    } else if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        ROS_ERROR("Failed to open socket.");
        return -1;
    }

    their_addr.sin_family = AF_INET;
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);

    if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
        ROS_ERROR("Failed to connect.");
        return -1;
    }

    i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
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

    if(i > MAXDATASIZE-40 || i < 0) {
        ROS_ERROR("Requested data too long.");
        return -1;
    }

    i += encode(buf+i, MAXDATASIZE-i-4, username.c_str(), password.c_str());
    if(i > MAXDATASIZE-4) {
        ROS_ERROR("Username and/or password too long.");
        return -1;
    }

    buf[i++] = '\r';
    buf[i++] = '\n';
    buf[i++] = '\r';
    buf[i++] = '\n';

    if(send(sockfd, buf, (size_t)i, 0) != i) {
        ROS_ERROR("Failed to send.");
        return -1;
    }

    int k = 0;
    int chunkymode = 0;
    int starttime = time(0);
    int lastout = starttime;
    int totalbytes = 0;
    int chunksize = 0;

    ROS_INFO("Start NTRIP client.");
    while(ros::ok() && !stop && !error && (numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) > 0) {
        if(!k) {
            buf[numbytes] = 0; /* latest end mark for strstr */

            if(numbytes > 17 &&
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
        std_msgs::ByteMultiArray msg;
        while(!stop && !cstop && !error && pos < numbytes) {
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
                    stop = 1;
                }
                break;
            case 4: /* output data */
                i = numbytes-pos;
                if(i > chunksize) {
                    i = chunksize;
                }

                msg.data.insert(msg.data.end(), &(buf[pos]), &(buf[pos + i]));
                dataTopic.publish(msg);

                //fwrite(buf+pos, (size_t)i, 1, stdout);

                totalbytes += i;
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


        /* overflow */
        if(totalbytes < 0) {
            totalbytes = 0;
            starttime = time(0);
            lastout = starttime;
        }

        int t = time(0);
        if(t > lastout + 5) {
            lastout = t;
            ROS_INFO("Bitrate is %dbyte/s.",
            totalbytes/(t - starttime));
        }
    }

    if(sockfd) {
        close(sockfd);
    }

    return 0;
}
