"""

 * nmpc_output_parse.py
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-10
 *
 * This file is part of vme-nmpc.
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

Description
===========

This module contains functions for parsing the output of vme-nmpc. For the most
part, the functions package data in lists. It is reccomended to cast them into
numpy arrays before doing any serious work. This is demonstrated in
analyse_output.py

"""

def is_number(s):
    """
    This function tests if a string can be represented by a float. If so, it
    returns true, and false otherwise.
    """
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse_welcome(infile, nmpc):
    """
    Parses the welcome message displayed my vme-nmpc. From there, it gets all
    of the information about the constants and parameters of the run.

    There is a caviat to be aware of when using this function. Because the last
    loop uses the presence of a '#' at the beginning of a line to mark the end
    of the welcome, that line is lost from the stream. It is returned by the
    function.

    The stream cannot be rewowned, because it is expected that the parsing
    functions can act on pipes.
    """
    line = infile.readline()
    if len(line) == 0:
            return 1
    while line[0] != '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
        if ("(T)" in line):
            nmpc["T"] = float(line.split()[-1])
        if ("(m)" in line):
            nmpc["m"] = int(line.split()[-1])
        if ("(n)" in line):
            nmpc["n"] = int(line.split()[-1])
        if ("(N)" in line):
            nmpc["N"] = int(line.split()[-1])
        if ("(C)" in line):
            nmpc["C"] = int(line.split()[-1])
        if ("(dg)" in line):
            nmpc["dg"] = float(line.split()[-1])
        if ("(cruising_speed)" in line):
            nmpc["cruising_speed"] = float(line.split()[-1])
        if ("(eps)" in line):
            nmpc["eps"] = float(line.split()[-1])
        if ("(Q0)" in line):
            nmpc["Q0"] = [float(line.split()[-2]), float(line.split()[-1])]
        if ("(Q)" in line):
            nmpc["Q"] = [float(line.split()[-2]), float(line.split()[-1])]
        if ("(R)" in line):
            nmpc["R"] = float(line.split()[-1])
        if ("(tgt)" in line):
            nmpc["tgt"] = [[], []]
            while 1:
                line = infile.readline()
                if len(line) == 0:
                    return 1
                if is_number(line.split()[0]):
                    nmpc["tgt"][0].append(float(line.split()[0]))
                    nmpc["tgt"][1].append(float(line.split()[1]))
                else:
                    break
        if ("(obst)" in line):
            nmpc["obst"] = [[], []]
            while 1:
                line = infile.readline()
                if len(line) == 0:
                    return 1
                if is_number(line.split()[0]):
                    nmpc["obst"][0].append(float(line.split()[0]))
                    nmpc["obst"][1].append(float(line.split()[1]))
                else:
                    break
        if ("(walls)" in line):
            nmpc["walls"] = []
            while 1:
                line = infile.readline()
                if len(line) == 0:
                    return 1
                if is_number(line.split()[0]):
                    nmpc["walls"].append([ \
                        float(line.split()[0]),
                        float(line.split()[1]),
                        float(line.split()[3]),
                        float(line.split()[4])])
                else:
                    break
    return line


def parse_sd_stats(infile, sd_loops, sd_loop_time):
    """
    This function gets the number of SD loops and the wall-clock time for the
    SD convergence which is displayed at the end of each SD loop.
    """
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    sd_loops.append(int(line.split()[0]))
    sd_loop_time.append(float(line.split()[1]))
    return 0


def parse_path_and_error(infile, nmpc, pne):
    """
    This function appends pne with a list of the data which is displayed in the
    'path and error' output block. These data include coordinates, velocity and
    the value of the error function of each point in the horizon.
    """
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    for k in range(0, nmpc["N"]-1):
        accum = []
        for element in line.split():
            accum.append(float(element))
        accum.pop(0)
        pne.append(accum)
        line = infile.readline()
        if len(line) == 0:
            return 1
    return 0


def parse_lagrangian_and_gradient(infile, nmpc, lgr):
    """
    This function parses the Langrange multiplier and gradient output block. It
    creates a list of the data, and appends the list to lgr.
    """
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    for k in range(0, nmpc["N"]-1):
        accum = []
        for element in line.split():
            accum.append(float(element))
        accum.pop(0)
        lgr.append(accum)
        line = infile.readline()
        if len(line) == 0:
            return 1
    return 0


def parse_tgt_wall_time(infile, tgt_time):
    """
    """
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    tgt_time.append([int(line.split()[0]), float(line.split()[1])])
    return 0


def get_block_meta(infile, meta):
    """
    Args:
        infile - the input stream.
        meta - the dictionary that will be populated.
    Return:
        1 - Failure.
        0 - Success.

    Caution: the meta data must be in the next line of the stream.

    """
    line = infile.readline()
    if len(line) == 0:
        return 0
    # Peel off the leading pound (#).
    line = line[1:]
    linesplit = line.split()
    for k in range(0, len(linesplit)):
        meta[linesplit[k]] = k