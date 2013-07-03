
"""
This function tests if a string can be represented by a float. If so, it returns
true, and false otherwise.
"""
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

"""
Parses the welcome message displayed my vme-nmpc. From there, it gets all of
the information about the constants and parameters of the run.

There is a caviat to be aware of when using this function. Because the last loop
uses the presence of a '#' at the beginning of a line to mark the end of the
welcome, that line is lost from the stream. It is returned by the function.

The stream cannot be rewowned, because it is expected that the parsing functions
can act on pipes.
"""
def parse_welcome(infile, nmpc):
    line = infile.readline()
    if len(line) == 0:
            return 1
    while line[0] != '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
        if ("(T)" in line):
            nmpc["T"] = float(line.split()[len(line.split()) - 1])
        if ("(m)" in line):
            nmpc["m"] = int(line.split()[len(line.split()) - 1])
        if ("(n)" in line):
            nmpc["n"] = int(line.split()[len(line.split()) - 1])
        if ("(N)" in line):
            nmpc["N"] = int(line.split()[len(line.split()) - 1])
        if ("(C)" in line):
            nmpc["C"] = int(line.split()[len(line.split()) - 1])
        if ("(dg)" in line):
            nmpc["dg"] = float(line.split()[len(line.split()) - 1])
        if ("(cruising_speed)" in line):
            nmpc["cruising_speed"] = float(line.split()[len(line.split()) - 1])
        if ("(eps)" in line):
            nmpc["eps"] = float(line.split()[len(line.split()) - 1])
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
    return line

"""
This function gets the number of SD loops and the wall-clock time for the SD
convergence which is displayed at the end of each SD loop.
"""
def parse_sd_stats(infile, sd_loops, sd_loop_time):
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

"""
This function appends pne with a list of the data which is displayed in the
'path and error' output block. These data include coordinates, velocity and the
value of the error function of each point in the horizon.
"""
def parse_path_and_error(infile, nmpc, pne):
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    for k in range(0, nmpc["N"]):
        accum = []
        for element in line.split():
            accum.append(float(element))
        accum.pop(0)
        pne.append(accum)
        line = infile.readline()
        if len(line) == 0:
            return 1
    return 0

"""
This function parses the Langrange multiplier and gradient output block. It
creates a list of the data, and appends the list to lgr.
"""
def parse_lagrangian_and_gradient(infile, nmpc, lgr):
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    for k in range(0, nmpc["N"]):
        accum = []
        for element in line.split():
            accum.append(float(element))
        accum.pop(0)
        lgr.append(accum)
        line = infile.readline()
        if len(line) == 0:
            return 1
    return 0
