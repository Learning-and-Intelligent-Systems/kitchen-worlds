
(define
  (problem none_240302_111202_default_1)
  (:domain domain)

  (:objects
	base
	base-torso
	left
	minifridge
	minifridge::joint_1
	minifridge::link_0
	right
	sink_counter_left
	veggiepotato
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm left)
	(arm right)

	(handempty left)
	(handempty right)

	(controllable left)

	(edible veggiepotato)

	(graspable veggiepotato)

	(space minifridge::link_0)

	(joint minifridge::joint_1)

	(oftype veggiepotato @edible)

	(staticlink minifridge::link_0)

	(bconf q48=(2.0, 4.0, 0.2, 3.142))

	(atbconf q48=(2.0, 4.0, 0.2, 3.142))

	(unattachedjoint minifridge::joint_1)

	(position minifridge::joint_1 pstn0=0.0)

	(atposition minifridge::joint_1 pstn0=0.0)
	(isjointto minifridge::joint_1 minifridge)
	(stackable veggiepotato sink_counter_left)

	(containable veggiepotato minifridge::link_0)

	(isclosedposition minifridge::joint_1 pstn0=0.0)

	(pose veggiepotato p0=(0.476, 3.832, 1.13, 0.0, -0.0, 1.643))

	(atpose veggiepotato p0=(0.476, 3.832, 1.13, 0.0, -0.0, 1.643))

	(aconf left aq144=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq288=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(defaultaconf left aq144=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultaconf right aq288=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(ataconf right aq288=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(ataconf left aq144=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(supported veggiepotato p0=(0.476, 3.832, 1.13, 0.0, -0.0, 1.643) sink_counter_left)

  )

  (:goal (and
    (in veggiepotato minifridge::link_0)
  ))
)
        