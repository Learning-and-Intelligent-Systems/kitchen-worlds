
(define
  (problem floating_pick)
  (:domain domain)

  (:objects
    braiserbody#1
	braiserbody#1::braiser_bottom
	counter#1
	counter#1::front_left_stove
	counter#1::hitman_tmp
	counter#1::indigo_tmp
	floor1
	hand
	meatturkeyleg#1
	oven#1
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(oven oven#1)

	(floor floor1)

	(handempty hand)

	(controllable hand)
	(counter counter#1)

	(braiserbody braiserbody#1)
	(graspable meatturkeyleg#1)

	(meatturkeyleg meatturkeyleg#1)

	(surface braiserbody#1::braiser_bottom)
	(surface counter#1::hitman_tmp)
	(surface counter#1::front_left_stove)
	(surface counter#1::indigo_tmp)

	(seconf q824=(0.9, 8, 0.7, 0, -1.571, 0))

	(atseconf q824=(0.9, 8, 0.7, 0, -1.571, 0))

	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::hitman_tmp)

	(pose meatturkeyleg#1 p0=((0.7, 8.506, 0.844), (0, 0, 0.095)))

	(atpose meatturkeyleg#1 p0=((0.7, 8.506, 0.844), (0, 0, 0.095)))

	(supported meatturkeyleg#1 p0=((0.7, 8.506, 0.844), (0, 0, 0.095)) counter#1::indigo_tmp)

  )

  (:goal (and
    (on meatturkeyleg#1 braiserbody#1::braiser_bottom)
  ))
)
        