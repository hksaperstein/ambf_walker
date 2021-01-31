#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
from StateMachines.States import *

class ExoStateMachine(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])

        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'Main'})

            smach.StateMachine.add('Main', Main(model, ["Poly", "DMP", "LowerBody", "walk", "LQR", "walkinit", "stairDMP", "Stand2Sit", "Sit2Stand"]),
                                   transitions={'Poly': 'Listening',
                                                'DMP': 'DMP',
                                                "LQR": "LQR",
                                                'LowerBody':'LowerBody',
                                                "stairDMP": "stairDMP",
                                                "walk": "walk",
                                                "walkinit": "WalkInit",
                                                "Stand2Sit": "Stand2Sit",
                                                "Sit2Stand": "Sit2Stand"})

            smach.StateMachine.add('LowerBody', LowerBody(model),
                                   transitions={'Lowering': 'LowerBody',
                                                'Lowered': 'Main'})

            smach.StateMachine.add('DMP', DMP(model),
                                   transitions={'stepping': 'DMP',
                                                'stepped': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('WalkInit', WalkInit(model),
                                   transitions={'WalkInitializing': 'WalkInit',
                                                'WalkInitialized': 'Main'},
                                   remapping={'q': 'q'})


            smach.StateMachine.add('LQR', LQR(model),
                                   transitions={'LQRing': 'LQR',
                                                'LQRed': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Listening', Listening(model),
                                   transitions={'Waiting': 'Listening',
                                                'Sending': 'Follow'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Follow', Follow(model),
                                   transitions={'Following': 'Follow',
                                                'Followed': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('stairDMP', StairDMP(model),
                                   transitions={'stairing': 'stairDMP',
                                                'staired': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('walk', Walk(model),
                                   transitions={'walking': 'walk',
                                                'walked': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Stand2Sit', Stand2Sit(model),
                                   transitions={'sitting': 'Stand2Sit',
                                                'sat': 'Sit2Stand'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Sit2Stand', Sit2Stand(model),
                                   transitions={'standing': 'Sit2Stand',
                                                'stood': 'Stand2Sit'},
                                   remapping={'q': 'q'})

        outcome = sm.execute()




# class ExoStateMachineFollowing(object):
#
#     def __init__(self, model):
#         sm = smach.StateMachine(outcomes=['outcome4'])
#         sm.userdata.counter = 0
#         with sm:
#             smach.StateMachine.add('Initialize', Initialize(model=model),
#                                     transitions={'Initializing': 'Initialize',
#                                                   'Initialized': 'Listening'}
#                                    )
#
#             smach.StateMachine.add('Listening', Listening(model),
#                                    transitions={'Waiting': 'Listening',
#                                                 'Sending': 'Follow'},
#                                    remapping={'count': 'count',
#                                               'q': 'q'})
#
#             smach.StateMachine.add('Follow', Follow(model),
#                                    transitions={'Following': 'Follow',
#                                                 'Followed': 'Listening'},
#                                    remapping={'count': 'count',
#                                               'q':'q'})
#
#         outcome = sm.execute()
#
#
#
# class ExoStateMachineGoTo(object):
#
#     def __init__(self, model):
#         sm = smach.StateMachine(outcomes=['outcome4'])
#         sm.userdata.counter = 0
#         with sm:
#             smach.StateMachine.add('Initialize', Initialize(model=model),
#                                     transitions={'Initializing': 'Initialize',
#                                                   'Initialized': 'GoTo'}
#                                    )
#
#             smach.StateMachine.add('GoTo', GoTo(model),
#                                    transitions={'Waiting': 'GoTo',
#                                                 'Sending': 'GoTo'})
#
#
#         outcome = sm.execute()
#
# class ExoStateMachineTest(object):
#
#     def __init__(self, model):
#         sm = smach.StateMachine(outcomes=['outcome4'])
#
#         with sm:
#             smach.StateMachine.add('Initialize', Initialize(model=model),
#                                     transitions={'Initializing': 'Initialize',
#                                                   'Initialized': 'Follow'})
#
#             smach.StateMachine.add('Follow', GMRTest(model),
#                                    transitions={'Following': 'Follow',
#                                                 'Followed': 'Follow'})
#
#         outcome = sm.execute()