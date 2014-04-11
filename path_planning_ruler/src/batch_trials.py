#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
from path_planning_ruler.move_base import *
from path_planning_ruler.parameterization import *
from path_planning_ruler.batch_arg_parser import parse_args
from path_planning_ruler.utils import *
from twilio_ros import send_text
import collections
import socket
from std_srvs.srv import *

basedir = '/home/dlu/Desktop/path_data'

def get_status_message(collected):
    host = socket.gethostname()
    ready = collected['total']-collected['to_run']+collected['run']
    return "Ran %d/%d tests (%d/%d) on %s"%(collected['run'], collected['to_run'], ready, collected['total'], host)

def delete_extra_models(gazebo):
    for mod in gazebo.get_models():
        if mod in ['plane1_model', 'pr2']:
            continue
        gazebo.delete_model(mod)
    
if __name__=='__main__':
    rospy.init_node('batch_trials_script')

    list_of_args, should_text = parse_args()
    stats = collections.defaultdict(int)
    
    print "Waiting for animator"
    rospy.wait_for_service('/animator/reset')
    resetter = rospy.ServiceProxy('/animator/reset', Empty)

    # Count Total Tests
    for args in list_of_args:
        parameterization = Parameterization(args.algorithm, args.scenario, args.variables, args.constants, basedir)
        stats['total'] += args.n * len(parameterization.parameterizations)
        for p in parameterization.parameterizations:
            if args.clean:
                stats['to_run'] += args.n
            else:
                for i in range(args.n):
                    fn = parameterization.get_full_filename(p, i)
                    if not os.path.exists(fn):
                        stats['to_run'] += 1
            if args.print_only:
                rospy.loginfo( parameterization.to_string(p) )

    if args.print_only:
        print "Need to run %d/%d tests"%(stats['to_run'], stats['total'])
        exit(0)
        
    print "Attempting to run %d/%d tests"%(stats['to_run'], stats['total'])

    g = GazeboHelper()
    delete_extra_models(g)

    # Run the tests
    for args in list_of_args:
        parameterization = Parameterization(args.algorithm, args.scenario, args.variables, args.constants, basedir)
        mkdir_p( parameterization.get_folder() )
        move_base = MoveBaseInstance(name=parameterization.node_name, quiet=args.quiet)

        for i in range(args.n):        
            for p in parameterization.parameterizations:
                parameterization.set_params(p)
                scenario = parameterization.scenario
                if len(parameterization.parameterizations)>1:
                    rospy.loginfo( parameterization.to_string(p) )

                g.quiet = args.quiet
                
                try:
                    filename = parameterization.get_full_filename(p, i)
                    if os.path.exists(filename) and not args.clean:
                        continue
                        
                    g.spawn_robot_maybe()
                    scenario.spawn(g)

                    rospy.loginfo('%s #%d/%d'%(scenario.key, i+1, args.n))
                   
                    try:
                        scenario.reset(g)
                        resetter()

                        move_base.start()
                        mb = MoveBaseClient(timeout=scenario.get_timeout())
                        if not mb.ready():
                            continue
                        mb.load_subscriptions()

                        t = rospy.Time.now()
                        data = mb.goto(scenario.get_goal_triple())
                        bag(filename, scenario.get_endpoints(t) + data)
                        stats['run'] += 1

                    finally:
                        move_base.shutdown()
                except rospy.service.ServiceException, e:
                    rospy.logerr("SERVICE EXCEPTION: %s" % str(e) )
                finally:
                    scenario.unspawn(g)

    s = get_status_message(stats)
    if should_text:
        send_text('+18455271217', s)
    print s

