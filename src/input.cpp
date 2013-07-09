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

#include "inc_errhandler.h"
#include "class_robot.h"
#include "struct_nmpc.h"
#include "struct_intok.h"
#include "struct_cl_opts.h"

void parse_command_line( int argc, char** argv, robot* vme, cl_opts* opts )
  {
    char *pvalue = NULL, *hvalue = NULL, *fvalue = NULL;
    int index;
    int c;
    char errnote[256];

    opterr = 0;

    while ( ( c = getopt(argc, argv, "vlp:h:f:") ) != -1)
      switch ( c )
        {
      case 'p' :
        vme->set_port(strtol(optarg, (char**) NULL, 10));
        break;
      case 'h' :
        vme->set_host(optarg);
        break;
      case 'f' :
        vme->set_configfile(optarg);
        break;
      case 'v' :
        opts->print_path_and_error = true;
        opts->print_lagrange_grad = true;
        break;
      case 'l' :
        opts->print_path_and_error = true;
        break;
      case '?' :
        if ( optopt == 'p' || optopt == 'h' || optopt == 'f' )
          {
            sprintf(errnote, "Option -%c: requires an argument.\n", optopt);
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
    /*
     * Read the file one character at a time while not at end of file...
     */
    while ( ( ch = fgetc(fd) ) != EOF )
      {
        /*
         * If we hit white space, gobble it, and keep track of new lines:
         */
        if ( isspace(ch) )
          {
            if ( ch == '\n' ) ++ ( *lineno );
            continue;
          }
        /*
         * Otherwise, if we hit a '#', gobble the rest of the line, and count
         * the new line:
         */
        else if ( ch == '#' )
          {
            while (ch != '\n')
              ch = fgetc(fd);
            ++ ( *lineno );
          }
        /*
         *  If we see a delimeter, we shouldn't expect one!
         */
        else if ( ch == '=' || ch == ':' )
          {
            sprintf(buffer, "Line: %d, Character: '%c'", *lineno, ch);
            report_error(RECOVERABLE_INPUT_FILE_SYNTAX, buffer);
            buffer[0] = '\0';
            return;
          }
        /*
         *  Made it through ignorable (new word?) stuff, so start recording the
         *  characters into the buffer:
         */
        else
          {
            *curb++ = ch;
            /*
             *  Any of these ends the recording:
             */
            while ( ( ch = fgetc(fd) ) != EOF && !isspace(ch) && ch != '='
                    && ch != ',' && ch != ':' && ch != ';')
              *curb++ = ch;
            /*
             *  We should expect that the recording was ended by these
             *  characters, otherwise, throw an error.
             */
            if ( ch != ' ' && ch != ',' && ch != ';' && ch != '\t' && ch != '='
                    && ch != '\n' )
              {
                /*
                 *  This error should really never happen unless we get \r or\v
                 *  or something else exoctic.
                 */
                sprintf(buffer,
                        "Line %d: Found invalid character '0x%x' ending a token.",
                        *lineno, ch);
                report_error(INVALID_INPUT_FILE_SYNTAX, buffer);
              }
            else
              {
                /*
                 *  By this point, a token has been recorded, and we have an
                 *  acceptible termination. Check to see if it was terminated
                 *  with a new line.
                 */
                if ( ch == '\n' ) lineno++;
                /*
                 *  If there is white space between the end of the token and its
                 *  delimeter, then gobble it.
                 */
                while (isspace(ch))
                  {
                    if ( ch == '\n' ) ++ ( *lineno );
                    ch = fgetc(fd);
                  }
                /*
                 *  Regardless of wheather or not the token was ended with white
                 *  space, or an accpeted delimeter, ch should hold the
                 *  delimeter at this point. Record it.
                 */
                if ( ch == '=' || ch == ',' || ch == ';' ) *lastdelim = ch;
                /*
                 *  Accept ':' as the same as '='. This behaviour may change
                 *  later, if I want '=' to be used for adjustable parameters,
                 *  and ':' if the code should keep them constant.
                 */
                if ( ch == ':' ) *lastdelim = '=';
              }
            break;
          }
      }
    /*
     *  Terminate the string in the buffer:
     */
    *curb = '\0';
    /*
     *  If we staid out of all of these decision trees, then ch could (should)
     *  be holding the EOF character for the file. Record it to let the calling
     *  routine know that we've hit the end of the file.
     */
    if ( ch == EOF )
      {
        *lastdelim = EOF;
        /*
         *  If the input file was made by a descent editor, there will be a
         *  newline character at the end of the file, which shouldn't count
         *  towards the line count. Bump it down.
         */
        -- ( *lineno );
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

/*!
 * Check the input token A to be sure it was seen in the input file. If yes, do
 * nothing, If no, throw a fetal error.
 */
void require_intok( intok* A )
  {
    char errmsg[256];
    if ( ! ( A->saw_tok ) )
      {
        sprintf(errmsg, "An entry for '%s' is required from input file\n",
                A->token);
        report_error(FATAL_INPUT_FILE_ERROR, errmsg);
      }
  }

/*
 * This function parses the input file.
 */
void parse_input_file( nmpc &controller, const char *infile )
  {
    int k = 0, lineno = 1;
    char *token, lastdelim;
    char buffer[BUFSIZ];
    char errmsg[256];
    FILE *infd;

    /*
     * List of items we expect to see in the input file. These bools will get
     * ticked as we go through the input file, and missing items can be handled
     * ex post facto.
     */
    intok tok_N = { "N", false };
    intok tok_C = { "C", false };
    intok tok_m = { "m", false };
    intok tok_n = { "n", false };
    intok tok_T = { "T", false };
    intok tok_dg = { "dg", false };
    intok tok_eps = { "eps", false };
    intok tok_tgttol = { "tgttol", false };
    intok tok_cruising_speed = { "cruising_speed", false };
    intok tok_R = { "R", false };
    intok tok_Q0 = { "Q0", false };
    intok tok_Q = { "Q", false };
    intok tok_S = { "S", false };
    intok tok_tgt = { "tgt", false };
    intok tok_obst = { "obst", false };

//  Begin...
    infd = fopen(infile, "r");
    if ( infd == NULL ) report_error(CANNOT_OPEN_INFILE, NULL);
    /*
     *  Loop, reading each record from the input file.
     */
    while (1)
      {
        /*
         *  Begin reading a record. The first token out should be the key that
         *  identifies which variable is being set. E.g. the 'T' in "T = 0.3;".
         */
        get_token(infd, buffer, &lastdelim, &lineno);
        if ( lastdelim == EOF ) break;
        /*
         *  The delim of the first token should be '=', otherwise something has
         *  gone wrong:
         */
        if ( lastdelim != '=' && lastdelim != EOF )
          {
            sprintf(errmsg,
                    "Line %d: Invalid field delimeter '%c', expected '=' or ':'.",
                    lineno, lastdelim);
            report_error(INVALID_INPUT_FILE_SYNTAX, errmsg);
          }
        /*
         *  Now that we should have the key, try to identify it and record the
         *  following tokens in their appropriate memory locations.
         *  With error checking!
         */
        if ( strcmp("T", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.T = atof(buffer);
            tok_T.saw_tok = true;
          }
        else if ( strcmp("N", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.N = atoi(buffer);
            tok_N.saw_tok = true;
          }
        else if ( strcmp("C", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.C = atoi(buffer);
            tok_C.saw_tok = true;
          }
        else if ( strcmp("m", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.m = atoi(buffer);
            tok_m.saw_tok = true;
          }
        else if ( strcmp("n", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.n = atoi(buffer);
            tok_n.saw_tok = true;
          }
        else if ( strcmp("dg", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.dg = atof(buffer);
            tok_dg.saw_tok = true;
          }
        else if ( strcmp("cruising_speed", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.cruising_speed = atof(buffer);
            tok_cruising_speed.saw_tok = true;
          }
        else if ( strcmp("tgttol", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.tgttol = atof(buffer);
            tok_tgttol.saw_tok = true;
          }
        else if ( strcmp("eps", buffer) == 0 )
          {
            get_token(infd, buffer, &lastdelim, &lineno);
            check_delim(lastdelim, ';', lineno);
            controller.eps = atof(buffer);
            tok_eps.saw_tok = true;
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
            tok_R.saw_tok = true;
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
            tok_Q0.saw_tok = true;
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
            tok_Q.saw_tok = true;
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
            tok_S.saw_tok = true;
          }
        else if ( strcmp("tgt", buffer) == 0 )
          {
            k = 0;
            controller.tgt = NULL;
            while (lastdelim != ';')
              {
                ++k;
                controller.tgt = (float*) realloc(controller.tgt,
                        k * sizeof(float));
                get_token(infd, buffer, &lastdelim, &lineno);
                controller.tgt[k - 1] = atof(buffer);
              }
            if ( k % 2 != 0 )
              {
                sprintf(errmsg,
                        "The odd number of entreis for tgt: cannot make pairs");
                report_error(ODD_NUMBER_TGT_COORDINATES, errmsg);
              };
            controller.ntgt = k / 2;
            tok_tgt.saw_tok = true;
          }
        else if ( strcmp("obst", buffer) == 0 )
          {
            k = 0;
            controller.obst = NULL;
            while (lastdelim != ';')
              {
                ++k;
                controller.obst = (float*) realloc(controller.obst,
                        k * sizeof(float));
                get_token(infd, buffer, &lastdelim, &lineno);
                controller.obst[k - 1] = atof(buffer);
              }

            if ( k % 2 != 0 )
              {
                sprintf(errmsg,
                        "The odd number of entreis for obst: cannot make pairs");
                report_error(ODD_NUMBER_OBST_COORDINATES, errmsg);
              };
            controller.nobst = k / 2;
            tok_obst.saw_tok = true;
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
    require_intok(&tok_N);
    require_intok(&tok_C);
    require_intok(&tok_m);
    require_intok(&tok_n);
    require_intok(&tok_T);
    require_intok(&tok_dg);
    require_intok(&tok_eps);
    require_intok(&tok_cruising_speed);
    require_intok(&tok_R);
    require_intok(&tok_Q0);
    require_intok(&tok_Q);
    require_intok(&tok_S);
    require_intok(&tok_tgttol);
    require_intok(&tok_tgt);
    require_intok(&tok_obst);
    fclose(infd);
  }
