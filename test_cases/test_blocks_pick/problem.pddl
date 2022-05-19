
(define
  (problem blocks_pick)
  (:domain domain)

  (:objects
    base
	cabbage
	left
	right
	table
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm right)
	(arm left)

	(handempty left)
	(handempty right)

	(supporter table)

	(controllable left)
	(graspable cabbage)

	(stackable cabbage table)

	(bconf q240=(1.79, 6, 3.142))

	(atbconf q240=(1.79, 6, 3.142))

	(pose cabbage p0=(4, 6, 0.95, 0, 0, 0))

	(atpose cabbage p0=(4, 6, 0.95, 0, 0, 0))

	(supported cabbage p0=(4, 6, 0.95, 0, 0, 0) table)

	(aconf left aq384=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq336=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(ataconf right aq336=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))
	(ataconf left aq384=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(defaultconf left aq384=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq336=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

  )

  (:goal (and
    (holding left cabbage)
  ))
)
        