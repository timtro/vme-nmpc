/*
 * robot.cpp
 * This file is part of vme-nmpc
 *
 * Copyright (C) 2013 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "struct_nmpc.h"
#include "inc_errhandler.h"
#include "class_robot.h"

#define M_PI_180 0.017453292519943295769236907684886f

robot::robot()
{
	poshead = (float*) calloc(3, sizeof(float));
	sockfd = 0;
	port = 5010;
	hostname = NULL;
	configfile = NULL;
}

robot::~robot()
{
//    free(hostname);
//    free(configfile);
	free(poshead);
}

void robot::set_host(char *host)
{
	hostname = host;
}

void robot::set_port(int portno)
{
	port = portno;
}

void robot::set_configfile(char *config)
{
	configfile = config;
}

char *
robot::conffile()
{
	return configfile;
}

int robot::tcp_connect()
{
	int n;
	struct sockaddr_in serv_addr;
	struct hostent *server;

	char buffer[256];

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		report_error(SOCK_CANNOT_CREATE_SOCK, NULL);

	server = gethostbyname(hostname);

	if (server == NULL)
		report_error(SOCK_CANNOT_CONNECT, NULL);

	memset((char *) &serv_addr, '\0', sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	memmove((char *) &serv_addr.sin_addr.s_addr, (char *) server->h_addr, server->h_length);
	serv_addr.sin_port = htons(port);
	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		report_error(SOCK_CANNOT_CONNECT, NULL);
	n = write(sockfd, "fd 0.5\n", 7);

	return 1;

}

/**
 * Simply send msg across the TCP/IP connection to the robot. The command should
 * be part of the Nav2 commander interface specification as implemented on the
 * target robot.
 */
int robot::Nav2(const char *msg)
{
	return write(sockfd, msg, strlen(msg));
}

/**
 * Send a formatted CLI command to the VirtualME. The command sets the
 * velocity and turn-rate of the robot (see Nav2 documentation for 'v
 * <n> <s> [<t>]'.
 */
int robot::Nav2_v(float* v, float *Dth)
{
	char msg[256];
	sprintf(msg, "\nv 0.000 %15f %15f\n", *v, *Dth / M_PI_180);
	fprintf(stderr, "%s", msg);
	return write(sockfd, msg, strlen(msg));
}

void robot::update_poshead(qnu* qu, const nmpc& C)
{
	int n;
	char buffer[256];

	n = write(sockfd, "q\n", 2);
	if (n < 0)
		report_error(SOCK_WRITE_ERROR, NULL);
	memset(buffer, '\0', 256);
	n = read(sockfd, buffer, 255);
	if (n < 0)
		report_error(SOCK_READ_ERROR, NULL);
	sscanf(buffer, "%f %f %f", &poshead[0], &poshead[1], &poshead[2]);
	qu[0].x = poshead[0];
	qu[0].y = poshead[1];
	qu[0].th = poshead[2] * M_PI_180;
	// TODO: Remember that C.C may be wrong, but the problem won't show until
	// variable speed in implemented.
	qu[0].Dx = cos(qu[0].th) * qu[C.C].v;
	qu[0].Dx = sin(qu[0].th) * qu[C.C].v;
}

