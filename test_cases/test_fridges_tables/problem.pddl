
(define
  (problem one_fridge_pr2_0922_090001)
  (:domain domain)

  (:objects
    None
	base
	base-torso
	cabinet
	cabinet::joint_0
	cabinet::joint_1
	cabinet::joint_2
	cabinet::link_3
	counter
	floor1
	left
	meatturkeyleg
	minifridge
	minifridge::joint_0
	minifridge::joint_1
	minifridge::joint_2
	minifridge::link_3
	right
	table
	veggietomato
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm right)
	(arm left)

	(wconf None)

	(floor floor1)
	(inwconf None)

	(object counter)
	(object minifridge)
	(object cabinet)
	(object table)

	(handempty right)
	(handempty left)

	(controllable left)
	(food veggietomato)
	(food meatturkeyleg)
	(supporter counter)
	(supporter table)

	(graspable veggietomato)
	(graspable meatturkeyleg)

	(door minifridge::joint_0)
	(door cabinet::joint_2)
	(door cabinet::joint_0)
	(door minifridge::joint_1)
	(door cabinet::joint_1)
	(door minifridge::joint_2)
	(space minifridge::link_3)
	(space cabinet::link_3)

	(joint minifridge::joint_0)
	(joint cabinet::joint_2)
	(joint cabinet::joint_0)
	(joint minifridge::joint_1)
	(joint cabinet::joint_1)
	(joint minifridge::joint_2)

	(stackable veggietomato table)
	(stackable meatturkeyleg table)
	(stackable veggietomato counter)
	(stackable meatturkeyleg counter)

	(isjointto cabinet::joint_2 cabinet)
	(isjointto minifridge::joint_0 minifridge)
	(isjointto minifridge::joint_1 minifridge)
	(isjointto cabinet::joint_0 cabinet)
	(isjointto cabinet::joint_1 cabinet)
	(isjointto minifridge::joint_2 minifridge)

	(bconf q600=(4.87, 2.928, 0.424, 1.002))
	(position minifridge::joint_2 pstn3=0.0)
	(position cabinet::joint_2 pstn0=1.522)
	(position cabinet::joint_0 pstn2=0.0)
	(position minifridge::joint_1 pstn1=0.785)
	(position minifridge::joint_0 pstn5=0.0)
	(position cabinet::joint_1 pstn4=0.0)

	(atbconf q600=(4.87, 2.928, 0.424, 1.002))
	(atposition minifridge::joint_2 pstn3=0.0)
	(atposition minifridge::joint_1 pstn1=0.785)
	(atposition cabinet::joint_2 pstn0=1.522)
	(atposition minifridge::joint_0 pstn5=0.0)
	(atposition cabinet::joint_0 pstn2=0.0)
	(atposition cabinet::joint_1 pstn4=0.0)

	(containable meatturkeyleg cabinet::link_3)
	(containable meatturkeyleg minifridge::link_3)
	(containable veggietomato cabinet::link_3)
	(containable veggietomato minifridge::link_3)

	(isclosedposition minifridge::joint_1 pstn1=0.785)
	(isclosedposition minifridge::joint_0 pstn5=0.0)
	(isclosedposition minifridge::joint_2 pstn3=0.0)
	(isclosedposition cabinet::joint_2 pstn0=1.522)
	(isclosedposition cabinet::joint_1 pstn4=0.0)
	(isclosedposition cabinet::joint_0 pstn2=0.0)

	(pose veggietomato p0=(3.304, 3.17, 1.138, 0.0, -0.0, 0.484))
	(pose meatturkeyleg p1=(3.284, 2.796, 1.098, 0.0, -0.0, 2.237))

	(atpose veggietomato p0=(3.304, 3.17, 1.138, 0.0, -0.0, 0.484))
	(atpose meatturkeyleg p1=(3.284, 2.796, 1.098, 0.0, -0.0, 2.237))

	(aconf left aq784=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq832=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(ataconf left aq784=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq832=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(defaultconf right aq832=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))
	(defaultconf left aq784=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))

	(contained meatturkeyleg p1=(3.284, 2.796, 1.098, 0.0, -0.0, 2.237) minifridge::link_3)
	(contained veggietomato p0=(3.304, 3.17, 1.138, 0.0, -0.0, 0.484) minifridge::link_3)

  )

  (:goal (and
    (in veggietomato cabinet::link_3)
  ))
)
        