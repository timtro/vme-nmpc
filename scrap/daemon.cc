/*
 * daemon.cpp
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
 *
 * This file is based on the work of J-P Gignac. It is basically a rewrite of
 * his daemon class from the telep-head source tree, found in
 *  telep-head/unwarp/Daemon.cpp
 */

#include <cstring>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <pthread.h>
#include <stdexcept>
#include <list>
#include "daemon.h"

Daemon::Daemon( int port, void (*request_handler)(int,void*), void *serveData)
   : sock(-1), request_handler_(request_handler), serveData(serveData)
{
   char buf[80];
   struct addrinfo *ai;
   struct addrinfo hints;
   int optval = 1;
   memset(&hints, 0, sizeof(hints));
   hints.ai_family = AF_UNSPEC;
   hints.ai_flags = AI_PASSIVE;
   snprintf(buf, sizeof(buf), "%d", port);
   getaddrinfo( NULL, buf, &hints, &ai);

   for (struct addrinfo* rp = ai; rp != NULL; rp = rp->ai_next) {
      sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (sock == -1) continue;
      setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
      if (bind(sock, rp->ai_addr, rp->ai_addrlen) == 0) break;
      close(sock);
   }

   freeaddrinfo(ai);
   if( sock == -1) {
      snprintf(buf, sizeof(buf), "Can't bind port %d", port);
      throw std::runtime_error(buf);
   }

   int rc = listen(sock, 1);
   if( rc == -1) {
      snprintf(buf, sizeof(buf),
               "Can't listen on port %d: %s",
               port, strerror(errno));
      close(sock);
      throw std::runtime_error(buf);
   }

   // Launch the server thread
   int err = pthread_create( &thread, NULL, daemon_thread, this);
   if( err != 0) {
      snprintf(buf, sizeof(buf), "Can't spawn thread: %s", strerror(err));
      close(sock);
      throw std::runtime_error(buf);
   }
}

Daemon::~Daemon()
{
   pthread_cancel( thread);
   pthread_join( thread, NULL);
   close(sock);
}

void *daemon_request_thread(void *ptr);

class RequestHandler {
   Daemon* d;
   int fd;
   pthread_t thread;
   friend void *daemon_request_thread(void *ptr);
   friend class RHResources;
   void (*(serve)())(int,void*)
   {
      return d->serve;
   }
   void *serveData()
   {
      return d->serveData;
   }
public:
   bool done;

   RequestHandler(Daemon* d) : d(d), done(false)
   {
      fd = accept( d->sock, NULL, NULL);
      if( fd == -1) throw std::runtime_error("Can't accept");
      if( pthread_create( &thread, NULL, daemon_request_thread, (void*)this)) {
         close(fd);
         throw std::runtime_error("Can't spawn thread");
      }
   }

   ~RequestHandler()
   {
      pthread_cancel(thread);
      pthread_join(thread, NULL);
   }
};

class RHResources {
   RequestHandler* rh;
public:
   RHResources(RequestHandler* rh) : rh(rh) {}
   ~RHResources()
   {
      close(rh->fd);
      rh->done = true;
   }
};

void *daemon_request_thread( void *ptr)
{
   RequestHandler* rh = ((RequestHandler*)ptr);
   RHResources rqc(rh);
   rh->serve()(rh->fd, rh->serveData());
   return NULL;
}

void *daemon_thread( void *ptr)
{
   Daemon* d = (Daemon*)ptr;
   std::list<RequestHandler> rdlist;

   for(;;) {
      // Create the request-handler.
      // Note: This operation blocks (waiting for a request)
      // and is a cancellation point.
      rdlist.emplace_back(d);

      // Clean up finished threads
      for( auto t=rdlist.begin(); t != rdlist.end();) {
         auto tt = t++;
         if( tt->done) rdlist.erase(tt);
      }
   }
   return NULL;
}


// For non-blocking sockets:
//
// #import <fcntl.h>

// /** Returns true on success, or false if there was an error */
// bool SetSocketBlockingEnabled(int fd, bool blocking)
// {
//    if (fd < 0) return false;

//    int flags = fcntl(fd, F_GETFL, 0);
//    if (flags < 0) return false;
//    flags = blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK);
//    return fcntl(fd, F_SETFL, flags);
// }