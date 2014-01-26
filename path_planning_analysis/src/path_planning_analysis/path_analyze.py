import os, os.path

def path_analyze(filename):
    FOLDER = 'path_data'
    M = {}
    abspath = os.path.abspath(filename)
    i = abspath.index(FOLDER) + len(FOLDER) + 1
    parts = abspath[i:].split('/')
    M['scenario'] = parts[0]
    fparts = os.path.splitext(parts[-1])[0].split('-')
    M['algorithm'] = fparts[0]
    M['trial'] = fparts[-1]
    if len(parts)==3:
        fields = parts[1].split('-')
        values = fparts[1:-1]
        if len(values)-1 > len(fields):
            flag = False
            nvals = []
            for v in values:
                if len(v)==0:
                    flag = True
                elif flag:
                    nvals.append("-%s"%v)
                    flag = False
                else:
                    nvals.append(v)
            values = nvals

        pvals = map(eval, values)

        M.update(zip(fields, pvals))
    return M
