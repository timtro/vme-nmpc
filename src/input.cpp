/*
 * input.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <vector>

#include "errinclude.h"
#include "robot.h"
#include "nmpc.h"

void parse_command_line( int argc, char **argv, robot *vme )
  {

    char *pvalue = NULL, *hvalue = NULL, *fvalue = NULL;
    int index;
    int c;
    char errnote[256];

    opterr = 0;

    while ((c = getopt(argc, argv, "p:h:f:")) != -1)
      switch ( c )
        {
      case 'p' :
        vme->set_port(strtol(optarg, (char **) NULL, 10));
        break;
      case 'h' :
        vme->set_host(optarg);
        break;
      case 'f' :
        vme->set_configfile(optarg);
        break;
      case '?' :
        if ( optopt == 'p' || optopt == 'h' || optopt == 'f' )
          {
            sprintf(errnote, "Option -%c: please specify input file name.\n",
                    optopt);
            report_error(CL_NO_ARG, errnote);
          }
        else if ( isprint(optopt) )
          {
            sprintf(errnote, "Unknown option `-%c'.\n", optopt);
            report_error(CL_INVALID_OPT, errnote);
          }
        else
          {
            sprintf(errnote, "Unknown option character `\\x%x'.\n", optopt);
            report_error(CL_UNKOWN_OPT_CHAR, errnote);
            return;
          }
        break;
      default :
        abort();
        }

  }

void get_token( FILE *fd, char *buffer, char *lastdelim, int *lineno )
  {
    char ch = '\0';
    char *eob = buffer + BUFSIZ - 1;
    char *curb = buffer;

    // Read the file one character at a time while not at end of file...
    //
    while ((ch = fgetc(fd)) != EOF)
      {
        // If we hit white space, gobble it, and keep track of new lines:
        //
        if ( ch == ' ' || ch == '\t' || ch == '\n' )
          {
            if ( ch == '\n' ) ++(*lineno);
            continue;
          }
        //
        // Otherwise, if we hit a '#', gobble the rest of the line, and count the
        // new line:
        //
        else if ( ch == '#' )
          {
            while (ch != '\n')
              ch = fgetc(fd);
            ++(*lineno);
          }
        //
        // If we see a delimeter, we shouldn't expect one!
        //
        else if ( ch == '=' || ch == ':' )
          {
            sprintf(buffer, "Line: %d, Character: '%c'", *lineno, ch);
            report_error(RECOVERABLE_INPUT_FILE_SYNTAX, buffer);
            buffer[0] = '\0';
            return;
          }
        //
        // Made it through ignormable (new word?) stuff, so start recording the
        // characters into the buffer:
        //
        else
          {
            *curb++ = ch;
            //
            // Any of these characters ends the recording:
            //
            while ((ch = fgetc(fd)) != EOF && ch != ' ' && ch != '\t'
                    && ch != '=' && ch != ',' && ch != ':' && ch != '\r'
                    && ch != ';' && ch != '\n')
              *curb++ = ch;
            //
            // We should expect that the recording was ended by these characters,
            // otherwise, throw an error.
            //
            if ( ch != ' ' && ch != ',' && ch != ';' && ch != '\t' && ch != '='
                    && ch != '\n' )
              {
                // This error should really never happen unless we get \r.
                sprintf(buffer,
                        "Line %d: Found invalid character '\\r' ending a token.",
                        *lineno, ch);
                report_error(INVALID_INPUT_FILE_SYNTAX, buffer);
              }
            else
              {
                //
                // By this point, a token has been recorded, and we have an
                // acceptible termination. Check to see if it was terminated with
                // a new line.
                //
                if ( ch == '\n' ) lineno++;
                //
                // If there is white space between the end of the token and its
                // delimeter, then gobble it.
                //
                while (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r')
                  {
                    if ( ch == '\n' ) ++(*lineno);
                    ch = fgetc(fd);
                  }
                //
                // Regardless of wheather or not the token was ended with white
                // space, or an accpeted delimeter, ch should hold the delimeter
                // at this point. Record it.
                //
                if ( ch == '=' || ch == ',' || ch == ';' ) *lastdelim = ch;
                //
                // Accept ':' as the same as '='. This behaviour may change later,
                // if I want '=' to be used for adjustable parameters, and ':' if
                // the code should keep them constant.
                //
                if ( ch == ':' ) *lastdelim = '=';
              }
            break;
          }
      }
    //
    // Terminate the string in the buffer:
    //
    *curb = '\0';
    //
    // If we staid out of all of these decision trees, then ch could (should)
    // be holding the EOF character for the file. Record it to let the calling
    // routine know that we've hit the end of the file.
    //
    if ( ch == EOF)
      {
        *lastdelim = EOF;
        //
        // If the input file was made by a descent editor, there will be a
        // newline character at the end of the file, which shouldn't count towards
        // the line count. Bump it down.
        //
        --(*lineno);
      }
  }

void check_delim( char lastdelim, char expected_delim, int lineno )
  {

    char errmsg[BUFSIZ];

    if ( lastdelim != expected_delim )
      {
        sprintf(errmsg, "Line %d: Expected ';' to end record. Got '%c'", lineno,
                lastdelim);
        report_error(FATAL_INPUT_FILE_SYNTAX, errmsg);
      }

  }

/*
 * This function parses the input file...
 *
 */
void parse_input_file( nmpc &controller, const char *infile )
  {
    int k = 0, lineno = 1;
    char *token, lastdelim;
    char buffer[BUFSIZ];
    char errmsg[256];
    FILE *infd;

    infd = fopen(infile, "r");
    if ( infd == NULL ) report_error(CANNOT_OPEN_INFILE, NULL);
    //
    // Loop, reading each record from the input file.
    //
    while (1)
      {
        // Begin reading a record. The first token out should be the key that
        // identifies which variable is being set. E.g. the 'T' in "T = 0.3;".
        //
        get_token(infd, buffer, &lastdelim, &lineno);
        if ( lastdelim == EOF) break;
        //
        // The delim of the first token should be '=', otherwise something has
        // gone wrong:
        //
        if ( lastdelim != '=' && lastdelim != EOF)
          {
            sprintf(errmsg,
                    "Line %d: Invalid field delimeter '%c', expected '=' or ':'.",
                    lineno, lastdelim);
            report_error(INVALID_INPUT_FILE_SYNTAX, errmsg);
          }
        //
        // Now that we should have the key, try to identify it and record the
        // following tokens in their appropriate memory locations.
        // With error checking!
        //
        if ( strcmp("T", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.T = atof(buffer);
          }
        else if ( strcmp("N", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.N = atoi(buffer);
          }
        else if ( strcmp("C", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.C = atoi(buffer);
          }
        else if ( strcmp("m", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.m = atoi(buffer);
          }
        else if ( strcmp("n", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.n = atoi(buffer);
          }
        else if ( strcmp("dg", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.dg = atof(buffer);
          }
        else if ( strcmp("cruising_speed", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.cruising_speed = atof(buffer);
          }
        else if ( strcmp("eps", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.eps = atof(buffer);
          }
        else if ( strcmp("R", buffer) == 0 )
          {
            controller.R = new float[2];
            for ( k = 0; k < controller.n; ++k )
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                if ( k == controller.n - 1 )
                  {
                    check_delim(lastdelim, ';', lineno);
                  }
                else
                  {
                    check_delim(lastdelim, ',', lineno);
                  }
                controller.R[k] = atof(buffer);
              }
          }
        else if ( strcmp("Q0", buffer) == 0 )
          {
            controller.Q0 = new float[2];
            for ( k = 0; k < 2; ++k )
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                if ( k == 1 )
                  {
                    check_delim(lastdelim, ';', lineno);
                  }
                else
                  {
                    check_delim(lastdelim, ',', lineno);
                  }
                controller.Q0[k] = atof(buffer);
              }
          }
        else if ( strcmp("Q", buffer) == 0 )
          {
            controller.Q = new float[2];
            for ( k = 0; k < 2; ++k )
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                if ( k == 1 )
                  {
                    check_delim(lastdelim, ';', lineno);
                  }
                else
                  {
                    check_delim(lastdelim, ',', lineno);
                  }
                controller.Q[k] = atof(buffer);
              }
          }
        else if ( strcmp("S", buffer) == 0 )
          {
            controller.S = new float[controller.m];
            for ( k = 0; k < controller.m; ++k )
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                if ( k == controller.m - 1 )
                  {
                    check_delim(lastdelim, ';', lineno);
                  }
                else
                  {
                    check_delim(lastdelim, ',', lineno);
                  }
                controller.S[k] = atof(buffer);
              }
          }
        else if ( strcmp("tgt", buffer) == 0 )
          {
            controller.tgt = new std::vector<float>(0);
            while (lastdelim != ';')
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                controller.tgt->push_back(atof(buffer));
              }
          }
        else if ( strcmp("obst", buffer) == 0 )
          {
            controller.obst = new std::vector<float>(0);
            while (lastdelim != ';')
              {
                get_token(infd, buffer, &lastdelim, &lineno);
                controller.obst->push_back(atof(buffer));
              }
          }
        else
          {
            sprintf(errmsg,
                    "WARNING: Line %d: Unrecognized key '%s'. Skipping record",
                    lineno, buffer);
            report_error(RECOVERABLE_INPUT_FILE_SYNTAX, errmsg);
            while (lastdelim != ';')
              get_token(infd, buffer, &lastdelim, &lineno);
          }
      }
    fclose(infd);
  }
