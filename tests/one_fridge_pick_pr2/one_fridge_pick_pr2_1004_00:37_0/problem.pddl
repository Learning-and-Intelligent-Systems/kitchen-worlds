
(define
  (problem test_fridges_tables)
  (:domain domain)

  (:objects
    none
	base
	base-torso
	cabinet
	cabinet::joint_0
	cabinet::link_1
	counter
	floor1
	left
	minifridge
	minifridge::joint_0
	minifridge::link_1
	right
	table
	veggiesweetpotato
	veggietomato
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm right)
	(arm left)

	(wconf none)

	(inwconf none)
	(floor floor1)

	(handempty left)
	(handempty right)

	(controllable left)
	(object minifridge)
	(object cabinet)
	(object counter)
	(object table)
	(supporter counter)
	(supporter table)

	(door cabinet::joint_0)
	(door minifridge::joint_0)

	(graspable veggietomato)
	(graspable veggiesweetpotato)
	(food veggiesweetpotato)
	(food veggietomato)

	(space minifridge::link_1)
	(space cabinet::link_1)

	(joint minifridge::joint_0)
	(joint cabinet::joint_0)

	(position cabinet::joint_0 pstn0=0.0)
	(position minifridge::joint_0 pstn1=0.0)
	(stackable veggiesweetpotato counter)
	(stackable veggiesweetpotato table)
	(stackable veggietomato table)
	(stackable veggietomato counter)

	(bconf q416=(4.976, 3.397, 0.438, 0.613))

	(atposition minifridge::joint_0 pstn1=0.0)
	(atposition cabinet::joint_0 pstn0=0.0)
	(isjointto minifridge::joint_0 minifridge)
	(isjointto cabinet::joint_0 cabinet)

	(atbconf q416=(4.976, 3.397, 0.438, 0.613))

	(containable veggiesweetpotato cabinet::link_1)
	(containable veggiesweetpotato minifridge::link_1)
	(containable veggietomato minifridge::link_1)
	(containable veggietomato cabinet::link_1)

	(isclosedposition minifridge::joint_0 pstn1=0.0)
	(isclosedposition cabinet::joint_0 pstn0=0.0)

	(pose veggiesweetpotato p0=(3.498, 2.723, 0.916, 0.0, -0.0, 2.765))
	(pose veggietomato p1=(3.142, 1.415, 0.893, -0.0, 0.0, 1.932))

	(atpose veggiesweetpotato p0=(3.498, 2.723, 0.916, 0.0, -0.0, 2.765))
	(atpose veggietomato p1=(3.142, 1.415, 0.893, -0.0, 0.0, 1.932))

	(aconf right aq560=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(aconf left aq512=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf left aq512=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq560=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(ataconf right aq560=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(ataconf left aq512=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(contained veggiesweetpotato p0=(3.498, 2.723, 0.916, 0.0, -0.0, 2.765) minifridge::link_1)
	(contained veggietomato p1=(3.142, 1.415, 0.893, -0.0, 0.0, 1.932) cabinet::link_1)

  )

  (:goal (and
    (holding left veggiesweetpotato)
  ))
)
        