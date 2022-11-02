
(define
  (problem test_feg_kitchen)
  (:domain domain)

  (:objects
    bottle#1
	bottle#1::joint_2
	bottle#2
	bottle#2::joint_2
	bottle#3
	bottle#3::joint_2
	floor1
	hand
	kitchenpot#1
	kitchenpot#1::joint_0
	meatturkeyleg
	meatturkeyleg#1
	meatturkeyleg#2
	meatturkeyleg#3
	meatturkeyleg#4
	microwave#1
	microwave#1::joint_0
	object#1
	object#1::joint_0
	object#1::joint_1
	object#2
	object#2::dishwasher_door
	object#3
	object#3::joint_0
	object#3::joint_1
	object#4
	object#4::joint_0
	object#4::knob_joint_1
	object#4::knob_joint_2
	object#4::knob_joint_3
	supporter#1
	supporter#2
	supporter#3
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(floor floor1)

	(handempty hand)

	(bottle bottle#1)
	(bottle bottle#3)
	(bottle bottle#2)
	(object object#3)
	(object meatturkeyleg#3)
	(object bottle#1)
	(object kitchenpot#1)
	(object bottle#3)
	(object meatturkeyleg)
	(object object#2)
	(object meatturkeyleg#2)
	(object object#4)
	(object meatturkeyleg#4)
	(object microwave#1)
	(object bottle#2)
	(object object#1)
	(object meatturkeyleg#1)

	(controllable hand)

	(food meatturkeyleg#1)
	(food meatturkeyleg#3)
	(food meatturkeyleg)
	(food meatturkeyleg#2)
	(food meatturkeyleg#4)

	(microwave microwave#1)
	(supporter supporter#3)
	(supporter supporter#2)
	(supporter supporter#1)

	(door object#4::joint_0)
	(door microwave#1::joint_0)
	(door object#4::knob_joint_2)
	(door object#3::joint_0)
	(door object#1::joint_1)
	(door object#4::knob_joint_1)
	(door object#4::knob_joint_3)
	(door object#3::joint_1)
	(door object#1::joint_0)

	(joint bottle#3::joint_2)
	(joint object#3::joint_0)
	(joint object#1::joint_1)
	(joint object#4::knob_joint_1)
	(joint object#2::dishwasher_door)
	(joint object#4::knob_joint_3)
	(joint object#3::joint_1)
	(joint object#1::joint_0)
	(joint bottle#2::joint_2)
	(joint bottle#1::joint_2)
	(joint kitchenpot#1::joint_0)
	(joint object#4::joint_0)
	(joint microwave#1::joint_0)
	(joint object#4::knob_joint_2)
	(oftype bottle#2 @bottle)
	(oftype bottle#3 @bottle)
	(oftype bottle#1 @bottle)
	(kitchenpot kitchenpot#1)

	(drawer object#2::dishwasher_door)
	(drawer bottle#2::joint_2)
	(drawer bottle#1::joint_2)
	(drawer kitchenpot#1::joint_0)
	(drawer bottle#3::joint_2)

	(isjointto object#4::joint_0 object#4)
	(isjointto object#4::knob_joint_1 object#4)
	(isjointto object#4::knob_joint_2 object#4)
	(isjointto object#4::knob_joint_3 object#4)
	(isjointto object#3::joint_1 object#3)
	(isjointto bottle#2::joint_2 bottle#2)
	(isjointto bottle#3::joint_2 bottle#3)
	(isjointto kitchenpot#1::joint_0 kitchenpot#1)
	(isjointto object#1::joint_0 object#1)
	(isjointto object#1::joint_1 object#1)
	(isjointto object#3::joint_0 object#3)
	(isjointto object#2::dishwasher_door object#2)
	(isjointto microwave#1::joint_0 microwave#1)
	(isjointto bottle#1::joint_2 bottle#1)
	(position object#4::joint_0 pstn9=0.0)
	(position object#4::knob_joint_1 pstn5=0.0)
	(position microwave#1::joint_0 pstn7=0.0)
	(position object#4::knob_joint_3 pstn6=0.0)
	(position object#2::dishwasher_door pstn3=0.0)
	(position bottle#1::joint_2 pstn4=0.0)
	(position bottle#3::joint_2 pstn1=0.0)
	(position object#3::joint_0 pstn10=0.0)
	(position object#1::joint_0 pstn8=0.0)
	(position kitchenpot#1::joint_0 pstn2=0.0)
	(position bottle#2::joint_2 pstn0=0.0)
	(position object#4::knob_joint_2 pstn11=0.0)
	(position object#3::joint_1 pstn13=0.0)
	(position object#1::joint_1 pstn12=0.0)

	(atposition bottle#2::joint_2 pstn0=0.0)
	(atposition object#3::joint_1 pstn13=0.0)
	(atposition object#1::joint_1 pstn12=0.0)
	(atposition object#4::knob_joint_2 pstn11=0.0)
	(atposition object#4::joint_0 pstn9=0.0)
	(atposition object#4::knob_joint_1 pstn5=0.0)
	(atposition microwave#1::joint_0 pstn7=0.0)
	(atposition bottle#1::joint_2 pstn4=0.0)
	(atposition bottle#3::joint_2 pstn1=0.0)
	(atposition object#2::dishwasher_door pstn3=0.0)
	(atposition object#1::joint_0 pstn8=0.0)
	(atposition object#4::knob_joint_3 pstn6=0.0)
	(atposition kitchenpot#1::joint_0 pstn2=0.0)
	(atposition object#3::joint_0 pstn10=0.0)

	(isclosedposition bottle#3::joint_2 pstn1=0.0)
	(isclosedposition microwave#1::joint_0 pstn7=0.0)
	(isclosedposition object#4::knob_joint_3 pstn6=0.0)
	(isclosedposition kitchenpot#1::joint_0 pstn2=0.0)
	(isclosedposition object#3::joint_0 pstn10=0.0)
	(isclosedposition object#1::joint_0 pstn8=0.0)
	(isclosedposition bottle#2::joint_2 pstn0=0.0)
	(isclosedposition object#3::joint_1 pstn13=0.0)
	(isclosedposition object#4::knob_joint_2 pstn11=0.0)
	(isclosedposition bottle#1::joint_2 pstn4=0.0)
	(isclosedposition object#2::dishwasher_door pstn3=0.0)
	(isclosedposition object#4::joint_0 pstn9=0.0)
	(isclosedposition object#1::joint_1 pstn12=0.0)
	(isclosedposition object#4::knob_joint_1 pstn5=0.0)

	(seconf q144=(5.0, 3.0, 0.7, 0.0, -1.571, 0.0))

	(atseconf q144=(5.0, 3.0, 0.7, 0.0, -1.571, 0.0))

	(originalseconf q144=(5.0, 3.0, 0.7, 0.0, -1.571, 0.0))

  )

  (:goal (and
    (holding hand bottle#3)
  ))
)
        