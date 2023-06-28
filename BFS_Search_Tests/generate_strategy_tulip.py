# -*- coding: utf-8 -*-
"""
Created on Thu Sep 26 16:44:37 2019

@author: badit
"""

#!/usr/bin/env python
"""Modification of test_eval_example2 except that when park=ON, always go to GOAL or if fuel runs out, go to refuel and then go to goal.
Repeat this when park is OFF. 
This example illustrates the use of TuLiP to synthesize a reactive
controller for a GR(1) specification.  We code the specification
directly in GR(1) form and then use TuLiP to synthesize a reactive
controller.

The system is modeled as a discrete transition system in which the
robot can be located anyplace on a 4x4 grid of cells with a refueling station:
    +----+----+----+----+
    | X1 | X2 | X3 | X4 |    Home: X16, Refuel: X8, Goal: X1
    +----+----+----+----+
    | X5 | X6 | X7 | X8 |
    +----+----+----+----+
    | X9 | X10| X11| X12|
    +----+----+----+----+
    | X13| X14| X15| X16|
    +----+----+----+----+

The robot is allowed to transition between any two adjacent cells;
diagonal motions are not allowed.  The robot should continuously
revisit the cell X1.

The environment consists of a single state called 'park' that
indicates that the robot should move to cell X1.

The system specification in its simplest form is given by
Hopefully, it automatically refuels...
Otherwise, revert back to example2.
  []<>park -> []<>X1 && [](!park -> <>X16) 

We must convert this specification into GR(1) form:

  env_init && []env_safe && []<>env_prog_1 && ... && []<>env_prog_m ->
      sys_init && []sys_safe && []<>sys_prog_1 && ... && []<>sys_prog_n
"""
# 21 Jul 2013, Richard M. Murray (murray@cds.caltech.edu)
# Import the packages that we need
# from __future__ import print_function
import logging
from tulip import spec
from tulip import synth
from tulip.transys import machines
from tulip import dumpsmach

logging.basicConfig(level=logging.WARNING)
#
# Environment specification
#
# The environment can issue a park signal that the robot must respond
# to by moving to the lower left corner of the grid.  We assume that
# the park signal is turned off infinitely often.
#
env_vars = {}
env_vars['park'] = 'boolean'
env_vars['Cr'] = (2,15)
env_init = {'(Cr = 2)'} 
# Car is not patrolling. It is moving anywhere in the center two columns
env_safe = {'(Cr = 14) -> X(Cr = 15 || Cr = 10)', '(Cr = 10) -> X(Cr = 14 || Cr = 6)', \
            '(Cr = 6) -> X(Cr = 10 || Cr = 2)', '(Cr = 2) -> X(Cr = 6 || Cr = 3)', \
            '(Cr = 3) -> X(Cr = 7 || Cr = 2)', '(Cr = 7) -> X(Cr = 3 || Cr = 11)', \
            '(Cr = 11) -> X(Cr = 7 || Cr = 15)', '(Cr = 15) -> X(Cr = 11 || Cr = 14)',
            } 
env_prog = {'park'} # []<>(park)

#
# System dynamics
#
# The system specification describes how the system is allowed to move
# and what the system is required to do in response to an environmental
# action.
# System can wait for the environment to get out of the way
sys_vars = {}
sys_vars['Xr'] = (1, 16)
sys_vars['fuel'] = (0,10)
sys_init = {'Xr=16', 'fuel = 10'}
# try and see if it is possible to let the vehicle move a little:
# that is, from Xr = 16, 
sys_safe = {'(Xr = 1) -> X (Xr=1 || Xr = 2 || Xr = 5)',
            '(Xr = 2) -> X (Xr = 1 || Xr = 3 || Xr = 6 || Xr = 2)',
            '(Xr = 3) -> X (Xr = 2 || Xr = 4 || Xr = 7 || Xr = 3)',
            '(Xr = 4) -> X (Xr = 4 || Xr = 3 || Xr = 8)',
            '(Xr = 5) -> X (Xr = 1 || Xr = 5 || Xr = 6 || Xr = 9)',
            '(Xr = 6) -> X(Xr = 2 || Xr = 5 || Xr = 7 || Xr = 10 || Xr = 6)',
            '(Xr = 7) -> X(Xr = 3 || Xr = 6 || Xr = 8 || Xr = 11 || Xr = 7)',
            '(Xr = 8) -> X(Xr = 4 || Xr = 7 || Xr = 8 || Xr = 12)',
            '(Xr = 9) -> X (Xr = 5 || Xr = 9 || Xr = 10 || Xr = 13)',
            '(Xr = 10) -> X (Xr = 6 || Xr = 9 || Xr = 11 || Xr = 14 || Xr = 10)',
            '(Xr = 11) -> X (Xr = 7 || Xr = 10 || Xr = 12 || Xr = 15 || Xr = 11)',
            '(Xr = 12) -> X (Xr = 8 || Xr = 11 || Xr = 12 || Xr = 16)',
            '(Xr = 13) -> X (Xr = 9 || Xr = 13 || Xr = 14)',
            '(Xr = 14) -> X(Xr = 10 || Xr = 13 || Xr = 15 || Xr = 14)',
            '(Xr = 15) -> X(Xr = 11 || Xr = 14 || Xr = 16 || Xr = 15)',
            '(Xr = 16) -> X(Xr = 12 || Xr = 15 || Xr = 16)',
            'Cr = 14 -> !(Xr = 14)',
            'Cr = 10 -> !(Xr = 10)',
            'Cr = 6 -> !(Xr = 6)',
            'Cr = 2 -> !(Xr = 2)',
            'Cr = 3 -> !(Xr = 3)',
            'Cr = 7 -> !(Xr = 7)',
            'Cr = 11 -> !(Xr = 11)',
            'Cr = 15 -> !(Xr = 15)',
            'fuel > 0',
            '(fuel = 10) <-> X(fuel = 9)',
            '(fuel = 9) <-> X(fuel = 8)',
            '(fuel = 8) <-> X(fuel = 7)',
            '(fuel = 7) <-> X(fuel = 6)',
            '(fuel = 6) <-> X(fuel = 5)',
            '(fuel = 5) <-> X(fuel = 4)',
            '(fuel = 4) <-> X(fuel = 3)',
            '(fuel = 3) <-> X(fuel = 2)',
            '(fuel = 2) <-> X(fuel = 1)',
            '(fuel = 1) -> X(fuel = 0) || (Xr = 4 && X(Xr = 8)) || (Xr = 7 && X(Xr = 8)) || (Xr = 12 && X(Xr = 8))',
            '(Xr = 8) -> (fuel = 10)',
}

sys_prog = set()                # empty set

# Environment won't crash into you:
for xi in range(2,16):
    env_safe |= {'(Xr = '+str(xi)+') -> X(!(Cr = '+str(xi)+'))'}
#
# System specification
#
# The system specification is that the robot should repeatedly revisit
# the upper right corner of the grid while at the same time responding
# to the park signal by visiting the lower left corner.  The LTL
# specification is given by
#
#     []<> X1 && [](!park -> <>X16) && [](!park -> <>X8)
#boolean
# Since this specification is not in GR(1) form, we introduce an
# environment variable X0reach that is initialized to True and the
# specification [](park -> <>X0) becomes
#
#     [](X (X16reach) <-> X16 || (X16reach && park)), []((X8reach && park) || X (X8reach) <-> X8))
#
# Augment the system description to make it GR(1)
sys_vars['X16reach'] = 'boolean'
sys_init |= {'X16reach'}
sys_safe |= {'(X (X16reach) <-> (Xr=16)) || (X16reach && park)'}
sys_prog |= {'X16reach', 'Xr=1'}

# Create a GR(1) specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
# Print specifications:
print(specs.pretty())

#
# Controller synthesis
#
# The controller decides based on current variable values only,
# without knowing yet the next values that environment variables take.
# A controller with this information flow is known as Moore.
specs.moore = True
# Ask the synthesizer to find initial values for system variables
# that, for each initial values that environment variables can
# take and satisfy `env_init`, the initial state satisfies
# `env_init /\ sys_init`.
specs.qinit = '\E \A'  # i.e., "there exist sys_vars: forall sys_vars"

# At this point we can synthesize the controller
# using one of the available methods.
strategy = synth.synthesize(specs)
assert strategy is not None, 'unrealizable'

# Generate a graphical representation of the controller for viewing, or a textual representation if pydot is missing.
# if not strategy.save('test_eval_example_modified.png'):
#     print(strategy)

# Writing strategy to file
for elem in env_init:
    break
elem = elem.strip('()').split()
env0 = int(elem[2])

if(env0 == 2):
    print("2")
    dumpsmach.write_python_case("TE2_v2.py", strategy, classname="TE_ctrl_init2")
elif(env0 == 3):
    print("3")
    dumpsmach.write_python_case("TE3_v2.py", strategy, classname="TE_ctrl_init3")
elif(env0 == 6):
    print("6")
    dumpsmach.write_python_case("TE6_v2.py", strategy, classname="TE_ctrl_init6")
elif(env0 == 7):
    print("7")
    dumpsmach.write_python_case("TE7_v2.py", strategy, classname="TE_ctrl_init7")
elif(env0 == 10):
    print("10")
    dumpsmach.write_python_case("TE10_v2.py", strategy, classname="TE_ctrl_init10")
elif(env0 == 11):
    print("11")
    dumpsmach.write_python_case("TE11_v2.py", strategy, classname="TE_ctrl_init11")
elif(env0 == 14):
    print("14")
    dumpsmach.write_python_case("TE14_v2.py", strategy, classname="TE_ctrl_init14")
elif(env0 == 15):
    print("15")
    dumpsmach.write_python_case("TE15_v2.py", strategy, classname="TE_ctrl_init15")
else:
    print('Keep the obstacle car initial position in the middle two columns')
    

## Generate a graph that represents the specifications set out by this file

