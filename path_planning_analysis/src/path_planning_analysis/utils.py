def to_strings(data, precision=3):
    table = []
    for row in data:
        strs = []
        for m in row:
            if type(m)==type(0.0):
                strs.append ( ("%%.%df"%precision) % m )
            else:
                strs.append(str(m))
        table.append(strs)
    return table

def print_table(data, spaces=True, precision=3):
    if spaces:
        for row in to_strings(data, precision):
            print '\t'.join(row)
    else:
        table = to_strings(data, precision)
        lens = [0]*len(table[0])
        for row in table:
            for i, value in enumerate(row):
                v = len(value)
                if v > lens[i]:
                    lens[i] = v

        for row in table:
            s = []
            for i, value in enumerate(row):
                n = lens[i]
                v = str(value)
                s.append( ("%%%ds"%n)%v )
            print '\t'.join(s)

