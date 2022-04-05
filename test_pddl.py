#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : test_pddl.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 04/05/2022
#
# Distributed under terms of the MIT license.

import jacinle
import os.path as osp
import lisdf.components as C
from lisdf.parsing import load_all


lisdf, domain, problem = load_all(
    "./assets/scenes/kitchen_lunch.lisdf",
    "./pddl/test_kitchen_lunch_domain.pddl",
    "./pddl/test_kitchen_lunch.pddl"
)

for t in domain.types.values():
    print(t)
for v in problem.init:
    print(v.predicate.to_pddl())
    for arg in v.arguments:
        if isinstance(arg, C.PDDLObject):
            print(" ", arg.name, arg.sdf_object)
        else:
            print(" ", arg.to_pddl())
