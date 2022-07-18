
(define
  (problem one_fridge_pr2_0718_100224)
  (:domain domain)

  (:objects
    base
	base-torso
	counter
	floor1
	left
	minifridge
	minifridge::joint_1
	minifridge::link_0
	right
	veggiecabbage#1
	wconf344
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm left)
	(arm right)

	(floor floor1)

	(handempty left)
	(handempty right)
	(wconf wconf344)

	(inwconf wconf344)

	(controllable left)
	(object minifridge)
	(object veggiecabbage#1)
	(object counter)
	(supporter counter)

	(door minifridge::joint_1)
	(space minifridge::link_0)

	(graspable veggiecabbage#1)
	(joint minifridge::joint_1)

	(veggiecabbage veggiecabbage#1)

	(stackable veggiecabbage#1 counter)

	(bconf q608=(4.819, 3.421, 0.477, 2.51))

	(atbconf q608=(4.819, 3.421, 0.477, 2.51))
	(isjointto minifridge::joint_1 minifridge)
	(position minifridge::joint_1 pstn0=2.095)

	(atposition minifridge::joint_1 pstn0=2.095)

	(containable veggiecabbage#1 minifridge::link_0)

	(isclosedposition minifridge::joint_1 pstn0=2.095)

	(pose veggiecabbage#1 p0=(3.161, 3.061, 0.837, 0.0, -0.0, 2.89))

	(atpose veggiecabbage#1 p0=(3.161, 3.061, 0.837, 0.0, -0.0, 2.89))

	(aconf right aq568=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))
	(aconf left aq296=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(ataconf right aq568=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))
	(ataconf left aq296=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(defaultconf left aq296=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq568=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(contained veggiecabbage#1 p0=(3.161, 3.061, 0.837, 0.0, -0.0, 2.89) minifridge::link_0)

  )

  (:goal (and
    (holding left veggiecabbage#1)
  ))
)
        