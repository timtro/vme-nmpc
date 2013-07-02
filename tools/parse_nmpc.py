

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


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
            nmpc["tgt"] = []
            while 1:
                line = infile.readline()
                if len(line) == 0:
                    return 1
                if is_number(line.split()[0]):
                    nmpc["tgt"].append([float(line.split()[0]),
                        float(line.split()[1])])
                else:
                    break
        if ("(obst)" in line):
            nmpc["obst"] = []
            while 1:
                line = infile.readline()
                if len(line) == 0:
                    return 1
                if is_number(line.split()[0]):
                    nmpc["obst"].append([float(line.split()[0]),
                        float(line.split()[1])])
                else:
                    break
    return 0


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


def parse_path_and_error(infile, pne):
    line = infile.readline()
    if len(line) == 0:
        return 1
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    while line[0] != '#':
        line = infile.readline()
        if len(line) == 0:
            return 1
    return 0
