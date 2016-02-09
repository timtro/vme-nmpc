/* This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
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

/* This file is based on the work of J.P. Gignac. It is basically a rewrite of
 * his daemon class from the telep-head source tree, found in
 * telep-head/unwarp/Daemon.cpp
 */

// TODO(TT): Wrap the server thread so that all pathways out make the thread
// joinable. Sugg: use Anthony Williams' scoped thread class.

#include "Daemon.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <list>
#include <thread>

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void daemon_threadfn(const Daemon* parentDaemon) {
  std::list<RequestTicket> ticketStubs;
  for (;;) {
    try {
      // NB: RequestTicket constructor calls accept() which blocks until a
      // connection is received.
      ticketStubs.emplace_back(parentDaemon);
    } catch (BlockedSocket) {
      // BlockedSocket indicates that Daemon is shutting down, so exit() the
      // thread.
      // std::exit(1);
      return;
    }

    ticketStubs.remove_if([](auto& each) { return each.done; });
  }
  return;
}

Daemon::Daemon(int port, std::function<void(int)> serverChild)
    : serverChild{serverChild}, shutdownFlag{false}, sockfd{-1} {
  char buf[80];
  struct addrinfo* ai;
  struct addrinfo hints;
  int optval = 1;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_flags = AI_PASSIVE;
  snprintf(buf, sizeof(buf), "%d", port);
  getaddrinfo(nullptr, buf, &hints, &ai);

  for (struct addrinfo* rp = ai; rp != nullptr; rp = rp->ai_next) {
    sockfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (sockfd == -1) continue;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    if (bind(sockfd, rp->ai_addr, rp->ai_addrlen) == 0) break;
    close(sockfd);
  }

  freeaddrinfo(ai);
  if (sockfd == -1) {
    snprintf(buf, sizeof(buf), "Can't bind port %d", port);
    throw std::runtime_error(buf);
  }

  int rc = listen(sockfd, 1);
  if (rc == -1) {
    snprintf(buf, sizeof(buf), "Can't listen on port %d: %s", port,
             strerror(errno));
    close(sockfd);
    throw std::runtime_error(buf);
  }

  try {
    daemonThread = std::thread(daemon_threadfn, this);
  } catch (...) {
    close(sockfd);
    throw;
  }
}

Daemon::~Daemon() {
  /* Shut down the socket to break daemonThread out of the accept() call so
   * that the thread can be properly joined.
   *
   * The shutdown flag must be true for safe shutdown, otherwise when accept()
   * returns -1, std::runtime error is thrown.
   */
  shutdownFlag = true;
  shutdown(sockfd, SHUT_RDWR);
  daemonThread.join();
  close(sockfd);
}

void server_child_wrapper(RequestTicket* ticket) {
  ticket->get_server_child()(ticket->connectionfd);
  close(ticket->connectionfd);
  ticket->done = true;
}

RequestTicket::RequestTicket(const Daemon* d) : parentDaemon{d}, done{false} {
  connectionfd = accept(parentDaemon->sockfd, nullptr, nullptr);
  if (connectionfd == -1) {
    /*
     * At this point, the failure of accept could be because the destructor
     * of Daemon has shutdown() the socket, or other reasons. Daemon's
     * destructor also sets its member shutdownFlag to true, so we can
     * distinguish.
     */
    if (d->shutdownFlag)
      throw BlockedSocket();
    else
      throw std::runtime_error("Daemon can't accept connection");
  }

  try {
    serverThread = std::thread(server_child_wrapper, this);
  } catch (...) {
    close(connectionfd);
    throw;
  }
}

// TODO: Check if this is what the original does
RequestTicket::~RequestTicket() { serverThread.join(); }
