
(define
  (problem None)
  (:domain domain)

  (:objects
    cabbage
	counter
	egg
	fridge
	left
	plate
	right
	salter
	sink
	stove
	table
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)
	(egg egg)

	(arm left)
	(arm right)

	(sink sink)

	(plate plate)
	(stove stove)
	(table table)

	(graspable egg)
	(graspable cabbage)
	(graspable salter)
	(graspable plate)
	(salter salter)

	(handempty left)
	(handempty right)
	(veggie cabbage)

	(counter counter)

	(supporter fridge)
	(supporter sink)
	(supporter stove)
	(supporter counter)
	(supporter table)

	(controllable left)

	(bconf q640=(0, 0, 0))
	(stackable egg fridge)
	(stackable egg sink)
	(stackable egg stove)
	(stackable egg counter)
	(stackable egg table)
	(stackable cabbage fridge)
	(stackable cabbage sink)
	(stackable cabbage stove)
	(stackable cabbage counter)
	(stackable cabbage table)
	(stackable salter fridge)
	(stackable salter sink)
	(stackable salter stove)
	(stackable salter counter)
	(stackable salter table)
	(stackable plate fridge)
	(stackable plate sink)
	(stackable plate stove)
	(stackable plate counter)
	(stackable plate table)

	(atbconf q640=(0, 0, 0))

	(pose egg p1=(2, -0.18, 0.95, 0))
	(pose cabbage p2=(2, 0, 0.95, 0))
	(pose salter p3=(2, 0.18, 0.95, 0))
	(pose plate p4=(2.18, 0, 0.95, 0))

	(atpose egg p1=(2, -0.18, 0.95, 0))
	(atpose cabbage p2=(2, 0, 0.95, 0))
	(atpose salter p3=(2, 0.18, 0.95, 0))
	(atpose plate p4=(2.18, 0, 0.95, 0))

	(supported egg p1=(2, -0.18, 0.95, 0) fridge)
	(supported cabbage p2=(2, 0, 0.95, 0) fridge)
	(supported salter p3=(2, 0.18, 0.95, 0) fridge)
	(supported plate p4=(2.18, 0, 0.95, 0) fridge)

	(aconf left aq832=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq736=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(ataconf left aq832=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq736=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(defaultconf left aq832=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq736=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

  )

  (:goal (and
    (cooked egg)
  ))
)
        