/*
 * Daemon.cpp
 * Author : Timothy A.V. Teatro
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

/* Derived works note (TT):
 * This file is based on the work of J.P. Gignac. It is basically a rewrite of
 * his daemon class from the telep-head source tree, found in
 * telep-head/unwarp/Daemon.cpp
 */
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


void daemon_threadfn(const Daemon* parent_daemon) {
  std::list<RequestTicket> ticket_stubs;
  for(;;) {
    try {
      // NB: RequestTicket constructor calls accept() which blocks until a
      // connection is received.
      ticket_stubs.emplace_back(parent_daemon);
    } catch(blocked_socket) {
      // blocked_socket indicates that Daemon is shutting down, so exit() the
      // thread.
      // std::exit(1);
      return;
    }

    ticket_stubs.remove_if([](auto& each) {
      return each.done;
    });
  }
  return;
}

Daemon::Daemon(int port, void (*server_child)(int))
  : sockfd_{-1}, server_child_{server_child}, shutdown_flag{false} {

  char buf[80];
  struct addrinfo *ai;
  struct addrinfo hints;
  int optval = 1;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_flags = AI_PASSIVE;
  snprintf(buf, sizeof(buf), "%d", port);
  getaddrinfo(nullptr, buf, &hints, &ai);

  for(struct addrinfo* rp = ai; rp != nullptr; rp = rp->ai_next) {
    sockfd_ = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if(sockfd_ == -1) continue;
    setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    if(bind(sockfd_, rp->ai_addr, rp->ai_addrlen) == 0) break;
    close(sockfd_);
  }

  freeaddrinfo(ai);
  if(sockfd_ == -1) {
    snprintf(buf, sizeof(buf), "Can't bind port %d", port);
    throw std::runtime_error(buf);
  }

  int rc = listen(sockfd_, 1);
  if(rc == -1) {
    snprintf(buf, sizeof(buf),
             "Can't listen on port %d: %s",
             port, strerror(errno));
    close(sockfd_);
    throw std::runtime_error(buf);
  }

  try {
    daemon_thread_ = std::thread(daemon_threadfn, this);
  } catch(...) {
    close(sockfd_);
    throw;
  }

}

Daemon::~Daemon() {
  /* Shut down the socket to break daemon_thread_ out of the accept() call so
   * that the thread can be properly joined.
   *
   * The shutdown flag must be true for safe shutdownbecause if it is false
   * when accept() returns -1, std::runtime error is thrown.
   */
  shutdown_flag = true;
  shutdown(sockfd_, SHUT_RDWR);
  daemon_thread_.join();
  close(sockfd_);
}

void server_child_wrapper(RequestTicket* ticket) {
  ticket->get_server_child()(ticket->connectionfd_);
  close(ticket->connectionfd_);
  ticket->done = true;
}

RequestTicket::RequestTicket(const Daemon* d) : parent_daemon_{d}, done{false} {
  connectionfd_ = accept(parent_daemon_->sockfd_, nullptr, nullptr);
  if(connectionfd_ == -1) {
    /*
     * At this point, the failure of accept could be because the destructor
     * of Daemon has shutdown() the socket, or other reasons. Daemon's
     * destructor also sets its member shutdown_flag to true, so we can
     * distinguish.
     */
    if(!d->shutdown_flag)
      throw std::runtime_error("Can't accept");
    else
      throw blocked_socket();
  }

  try {
    server_thread_ = std::thread(server_child_wrapper, this);
  } catch(...) {
    close(connectionfd_);
    throw;
  }
}

//TODO: Theck if this is what the original does
RequestTicket::~RequestTicket() {
  server_thread_.join();
}