
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

	(bconf q336=(0, 0, 0))
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

	(atbconf q336=(0, 0, 0))

	(pose egg p0=(2, -0.18, 0.95, 0))
	(pose cabbage p1=(2, 0, 0.95, 0))
	(pose salter p2=(2, 0.18, 0.95, 0))
	(pose plate p3=(2.18, 0, 0.95, 0))

	(atpose egg p0=(2, -0.18, 0.95, 0))
	(atpose cabbage p1=(2, 0, 0.95, 0))
	(atpose salter p2=(2, 0.18, 0.95, 0))
	(atpose plate p3=(2.18, 0, 0.95, 0))

	(supported egg p0=(2, -0.18, 0.95, 0) fridge)
	(supported cabbage p1=(2, 0, 0.95, 0) fridge)
	(supported salter p2=(2, 0.18, 0.95, 0) fridge)
	(supported plate p3=(2.18, 0, 0.95, 0) fridge)

	(aconf left aq528=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq432=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(ataconf left aq528=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq432=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(defaultconf left aq528=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq432=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

  )
    
  (:goal (and 
    (served cabbage)
  ))
)
        