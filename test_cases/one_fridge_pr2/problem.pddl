
(define
  (problem one_fridge_pr2)
  (:domain domain)

  (:objects
    base
	base-torso
	counter
	floor1
	left
	minifridge
	minifridge::joint_1
	minifridge::joint_2
	minifridge::link_0
	right
	veggiegreenpepper
	wconf728
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm left)
	(arm right)

	(floor floor1)

	(object counter)
	(object minifridge)
	(wconf wconf728)

	(handempty right)
	(handempty left)

	(inwconf wconf728)

	(controllable left)
	(supporter counter)

	(food veggiegreenpepper)

	(door minifridge::joint_2)
	(door minifridge::joint_1)
	(space minifridge::link_0)

	(joint minifridge::joint_1)
	(joint minifridge::joint_2)

	(graspable veggiegreenpepper)

	(bconf q64=(5, 3, 0.2, 3.142))

	(atbconf q64=(5, 3, 0.2, 3.142))

	(stackable veggiegreenpepper counter)

	(position minifridge::joint_2 pstn1=0)
	(position minifridge::joint_1 pstn0=0)

	(atposition minifridge::joint_1 pstn0=0)
	(atposition minifridge::joint_2 pstn1=0)

	(isjointto minifridge::joint_1 minifridge)
	(isjointto minifridge::joint_2 minifridge)

	(isclosedposition minifridge::joint_2 pstn1=0)
	(isclosedposition minifridge::joint_1 pstn0=0)

	(containable veggiegreenpepper minifridge::link_0)

	(pose veggiegreenpepper p0=(2.99, 2.89, 0.589, 0, 0, 1.943))

	(atpose veggiegreenpepper p0=(2.99, 2.89, 0.589, 0, 0, 1.943))

	(aconf left aq552=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq648=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(ataconf left aq552=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq648=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(defaultconf right aq648=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))
	(defaultconf left aq552=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(contained veggiegreenpepper p0=(2.99, 2.89, 0.589, 0, 0, 1.943) minifridge::link_0)

  )

  (:goal (and
    (openedjoint minifridge::joint_1)
  ))
)
        