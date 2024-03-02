
(define
  (problem none_240302_110822_default_0)
  (:domain domain)

  (:objects
	base
	base-torso
	left
	minifridge
	minifridge::joint_0
	right
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm right)
	(arm left)

	(handempty right)
	(handempty left)

	(controllable left)

	(joint minifridge::joint_0)

	(bconf q992=(2.0, 4.0, 0.2, 3.142))

	(atbconf q992=(2.0, 4.0, 0.2, 3.142))
	(unattachedjoint minifridge::joint_0)

	(position minifridge::joint_0 pstn0=0.0)

	(atposition minifridge::joint_0 pstn0=0.0)
	(isjointto minifridge::joint_0 minifridge)

	(isclosedposition minifridge::joint_0 pstn0=0.0)

	(aconf right aq136=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(aconf left aq88=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(ataconf right aq136=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(ataconf left aq88=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(defaultaconf right aq136=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(defaultaconf left aq88=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

  )

  (:goal (and
    (graspedhandle minifridge::joint_0)
  ))
)
        