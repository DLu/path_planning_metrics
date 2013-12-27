import argparse
import sys

def main_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', metavar='algorithm.cfg')
    parser.add_argument('scenario', metavar='scenario.yaml')
    parser.add_argument('-v', dest='variables', nargs="+", type=str)
    parser.add_argument('-c', dest='constants', nargs="+", type=str)
    parser.add_argument("-n", "--num_trials", dest="n", help='Number of trials per configuration', metavar='N', type=int, default=10)
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('-q', '--quiet', action='store_true')
    parser.add_argument('-t', '--text', action='store_true')
    parser.add_argument('-p', '--print_only', action='store_true')
    return parser

def batch_parser():
    p2 = argparse.ArgumentParser()
    p2.add_argument('-b', '--batch', dest="batchfile")
    p2.add_argument('-q', '--quiet', action='store_true')
    p2.add_argument('-c', '--clean', action='store_true')
    p2.add_argument('-t', '--text', action='store_true')
    return p2


def parse_args():
    text = False
    list_of_args = [] 
    parser = main_parser()

    if '-b' in sys.argv:
        args = batch_parser().parse_args()
        text = args.text
        f = open(args.batchfile, 'r')
        for line in f.readlines():
            if len(line.strip())==0:
                continue
            list_of_args.append( parser.parse_args(line.split()) )
        f.close()        
    else:
        args = parser.parse_args() 
        text = args.text
        list_of_args.append( args )

    return list_of_args, text


