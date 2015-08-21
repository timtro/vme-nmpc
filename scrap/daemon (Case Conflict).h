/*
 * vme-nmpc/src/Daemon.h
 * Author : Timothy A.V. Teatro and John-Paul Gignac
 * Date   : 2015-07-29
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

/* Derived works note (T.T.):
 *
 * This file is based on the work of J-P Gignac. It is basically a rewrite of
 * his daemon class from the telep-head source tree, found in
 * 	telep-head/unwarp/Daemon.cpp
 */

#ifndef VME_NMPC_SRC_DAEMON_H_
#define VME_NMPC_SRC_DAEMON_H_

/**
 * For historical AND practical reasons, I use the name daemon instead of
 * server. The word server in this object refers to the functions which handle
 * the _serving_ of requests while the daemon is the process that runs in the
 * background creating and maintaining the socket.
 *
 * An object of type Daemon will spawn a thread that runs daemon_thread(). The
 * constructor of Daemon requires a port number and a request_handler function.
 * The Daemon object will create and bind a socket and listen on the specified
 * port.
 *
 * The thread function daemon_thread() maintains a list, ticket_stubs, of
 * objects of type RequestTicket. The function runs in a loop and in each
 * iteration, it emplaces_back() a new RequestTicket on the list. The
 * constructor of a RequestTicket calls accept(), which blocks execution in
 * the thread until a connection is made. Once the connection is made, the
 * construction continues wherein a new thread is spawned which runs
 * server_threadfn() which executes the request_handler() function. When the
 * server thread terminates, the RequestTicket is marked done and is cleaned
 * up in the next iteration of daemon_thread().
 *
 */

#include <pthread.h>

void *daemon_thread( void *ptr);
class RequestTicket;

// class Daemon {
//   int sockfd_;
//   pthread_t thread;
//   void (*request_handler_)(int,void*);
//   void* request_handler_data_;
//   friend void *daemon_thread( void *ptr);
//   friend class RequestTicket;

//  protected:
//   void shutdown();

//  public:
//   Daemon( int port, void (*request_handler_)(int,void*),
//           void *request_handler_data_);
//   ~Daemon();
// };

void *daemon_thread( void *ptr);
class RequestHandler;

class Daemon {
  int sock;
  pthread_t thread;
  void (*serve)(int,void*);
  void* serveData;
  friend void *daemon_thread( void *ptr);
  friend class RequestHandler;

 protected:
  void shutdown();

 public:
  Daemon( int port, void (*serve)(int,void*), void *serveData);
  ~Daemon();
};

#endif // VME_NMPC_SRC_DAEMON_H_