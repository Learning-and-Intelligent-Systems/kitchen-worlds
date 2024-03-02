
(define
  (problem none_240302_111021_default_0)
  (:domain domain)

  (:objects
	base
	base-torso
	left
	minifridge
	minifridge::joint_1
	right
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm left)
	(arm right)

	(handempty right)
	(handempty left)

	(controllable left)

	(joint minifridge::joint_1)

	(bconf q344=(2.0, 4.0, 0.2, 3.142))

	(atbconf q344=(2.0, 4.0, 0.2, 3.142))
	(unattachedjoint minifridge::joint_1)

	(position minifridge::joint_1 pstn0=0.0)

	(atposition minifridge::joint_1 pstn0=0.0)
	(isjointto minifridge::joint_1 minifridge)

	(isclosedposition minifridge::joint_1 pstn0=0.0)

	(aconf left aq440=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq488=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(defaultaconf left aq440=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultaconf right aq488=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(ataconf right aq488=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(ataconf left aq440=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

  )

  (:goal (and
    (graspedhandle minifridge::joint_1)
  ))
)
        