
(define
  (problem test_feg_closed_doors)
  (:domain domain)

  (:objects
    basin#1
	basin#1::basin_bottom
	braiserbody#1
	braiserbody#1::braiser_bottom
	braiserlid#1
	counter#1
	counter#1::chewie_door_left_joint
	counter#1::chewie_door_right_joint
	counter#1::dagger
	counter#1::dagger_door_left_joint
	counter#1::dagger_door_right_joint
	counter#1::front_left_stove
	counter#1::hitman_drawer_top
	counter#1::hitman_tmp
	counter#1::indigo_tmp
	counter#1::sektion
	dishwasher#1
	faucet#1
	faucet#1::joint_faucet_0
	faucet#1::joint_faucet_1
	floor1
	fridge#1
	fridge#1::fridge_door
	fridge#1::shelf_bottom
	hand
	meatturkeyleg#1
	oven#1
	oven#1::knob_joint_2
	veggiecabbage#1
	vinegarbottle#1
	oilbottle#1
	wconf0
	wconf184
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

  (basin basin#1)
	(floor floor1)

	(handempty hand)
	(wconf wconf184)
	(wconf wconf0)

	(inwconf wconf184)

	(controllable hand)

	(cleaned meatturkeyleg#1)

	(edible meatturkeyleg#1)
	(edible veggiecabbage#1)

	(graspable braiserlid#1)
	(graspable veggiecabbage#1)
	(graspable oilbottle#1)
	(graspable meatturkeyleg#1)
	(graspable vinegarbottle#1)

	(knob oven#1::knob_joint_2)
	(knob faucet#1::joint_faucet_1)
	(knob faucet#1::joint_faucet_0)

	(door fridge#1::fridge_door)
	(door counter#1::dagger_door_left_joint)
	(door counter#1::chewie_door_left_joint)
	(door counter#1::dagger_door_right_joint)
	(door counter#1::chewie_door_right_joint)

  (joint fridge#1::fridge_door)
	(joint oven#1::knob_joint_2)
	(joint faucet#1::joint_faucet_0)
	(joint faucet#1::joint_faucet_1)
	(joint counter#1::dagger_door_left_joint)
	(joint counter#1::chewie_door_left_joint)
	(joint counter#1::dagger_door_right_joint)
	(joint counter#1::chewie_door_right_joint)

	(oftype veggiecabbage#1 @edible)
	(oftype oilbottle#1 @bottle)
	(oftype meatturkeyleg#1 @edible)
	(oftype vinegarbottle#1 @bottle)

	(space counter#1::sektion)
	(space counter#1::hitman_drawer_top)
	(space counter#1::dagger)

	(surface counter#1::front_left_stove)
	(surface fridge#1::shelf_bottom)
	(surface counter#1::indigo_tmp)
	(surface braiserbody#1::braiser_bottom)
	(surface basin#1::basin_bottom)

	(cleaningsurface basin#1::basin_bottom)

	(position oven#1::knob_joint_2 pstn1=0)
	(position fridge#1::fridge_door pstn5=1.907)
	(position fridge#1::fridge_door pstn0=0)
	(position faucet#1::joint_faucet_0 pstn3=0)
	(position faucet#1::joint_faucet_1 pstn2=0)
	(position counter#1::chewie_door_right_joint pstn15=0)
	(position counter#1::dagger_door_left_joint pstn17=0)
	(position counter#1::dagger_door_right_joint pstn12=1.57)
	(position counter#1::chewie_door_left_joint pstn10=-1.57)
	(position counter#1::dagger_door_right_joint pstn16=0)
	(position counter#1::dagger_door_left_joint pstn13=-1.57)
	(position counter#1::chewie_door_left_joint pstn14=0)
	(position counter#1::chewie_door_right_joint pstn11=1.57)

	(containable braiserlid#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::dagger)
	(containable veggiecabbage#1 counter#1::hitman_drawer_top)
	(containable braiserlid#1 counter#1::sektion)
	(containable braiserlid#1 counter#1::dagger)
	(containable veggiecabbage#1 counter#1::dagger)
	(containable vinegarbottle#1 counter#1::hitman_drawer_top)
	(containable vinegarbottle#1 counter#1::sektion)
	(containable veggiecabbage#1 counter#1::sektion)
	(containable vinegarbottle#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::hitman_drawer_top)
	(containable meatturkeyleg#1 counter#1::sektion)
	(containable meatturkeyleg#1 counter#1::dagger)
	(containable oilbottle#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::sektion)

	(seconf q40=(1.2, 8, 1.2, 0, -1.571, 0))
	(atseconf q40=(1.2, 8, 1.2, 0, -1.571, 0))
  (originalseconf q40=(1.2, 8, 1.2, 0, -1.571, 0))

	(atposition fridge#1::fridge_door pstn0=0)
	(atposition faucet#1::joint_faucet_0 pstn3=0)
	(atposition oven#1::knob_joint_2 pstn1=0)
	(atposition faucet#1::joint_faucet_1 pstn2=0)
	(atposition counter#1::chewie_door_left_joint pstn14=0)
	(atposition counter#1::dagger_door_left_joint pstn17=0)
	(atposition counter#1::dagger_door_right_joint pstn16=0)
	(atposition counter#1::chewie_door_right_joint pstn15=0)

	(isjointto faucet#1::joint_faucet_1 faucet#1)
	(isjointto oven#1::knob_joint_2 oven#1)
	(isjointto fridge#1::fridge_door fridge#1)
	(isjointto faucet#1::joint_faucet_0 faucet#1)
	(isjointto counter#1::dagger_door_right_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)
	(isjointto counter#1::dagger_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_left_joint counter#1)

	(heatingsurface braiserbody#1::braiser_bottom)

  (stackable veggiecabbage#1 counter#1::hitman_tmp)
  (stackable oilbottle#1 counter#1::indigo_tmp)
  (stackable meatturkeyleg#1 counter#1::indigo_tmp)
  (stackable vinegarbottle#1 counter#1::front_left_stove)
  (stackable vinegarbottle#1 braiserbody#1::braiser_bottom)
  (stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
  (stackable vinegarbottle#1 counter#1::hitman_tmp)
  (stackable oilbottle#1 counter#1::front_left_stove)
  (stackable meatturkeyleg#1 counter#1::front_left_stove)
  (stackable oilbottle#1 counter#1::hitman_tmp)
  (stackable meatturkeyleg#1 counter#1::hitman_tmp)
  (stackable braiserlid#1 counter#1::indigo_tmp)
  (stackable oilbottle#1 braiserbody#1::braiser_bottom)
  (stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
  (stackable vinegarbottle#1 counter#1::indigo_tmp)
  (stackable braiserlid#1 counter#1::front_left_stove)
  (stackable veggiecabbage#1 counter#1::indigo_tmp)
  (stackable braiserlid#1 counter#1::hitman_tmp)
  (stackable veggiecabbage#1 counter#1::front_left_stove)
  (stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 basin#1::basin_bottom)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable braiserlid#1 basin#1::basin_bottom)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable oilbottle#1 basin#1::basin_bottom)

	(isclosedposition faucet#1::joint_faucet_1 pstn2=0)
	(isclosedposition faucet#1::joint_faucet_0 pstn3=0)
	(isclosedposition fridge#1::fridge_door pstn0=0)
	(isclosedposition oven#1::knob_joint_2 pstn1=0)
	(isclosedposition counter#1::chewie_door_left_joint pstn14=0)
	(isclosedposition counter#1::dagger_door_left_joint pstn17=0)
	(isclosedposition counter#1::dagger_door_right_joint pstn16=0)
	(isclosedposition counter#1::chewie_door_right_joint pstn15=0)

  (isopenedposition fridge#1::fridge_door pstn5=1.907)
	(isopenedposition counter#1::dagger_door_right_joint pstn12=1.57)
	(isopenedposition counter#1::chewie_door_left_joint pstn10=-1.57)
	(isopenedposition counter#1::dagger_door_left_joint pstn13=-1.57)
	(isopenedposition counter#1::chewie_door_right_joint pstn11=1.57)

  (atpose oilbottle#1 p11=(0.324, 8.944, 1.555, 0, 0, 1.615))
	(atpose vinegarbottle#1 p10=(0.322, 8.631, 1.526, 0, 0, 2.383))
	(atpose braiserlid#1 p0=(0.557, 8.211, 0.821, 0, 0, 0.627))
	(atpose meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126))
	(atpose veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536))

  (pose oilbottle#1 p11=(0.324, 8.944, 1.555, 0, 0, 1.615))
	(pose vinegarbottle#1 p10=(0.322, 8.631, 1.526, 0, 0, 2.383))
	(pose meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126))
	(pose veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536))
	(pose braiserlid#1 p0=(0.557, 8.211, 0.821, 0, 0, 0.627))

	(controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)
	(controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)

	(newwconfpst wconf184 fridge#1::fridge_door pstn5=1.907 wconf0)

	(contained vinegarbottle#1 p10=(0.322, 8.631, 1.526, 0, 0, 2.383) counter#1::dagger)
	(contained oilbottle#1 p11=(0.324, 8.944, 1.555, 0, 0, 1.615) counter#1::dagger)

	(supported meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536) fridge#1::shelf_bottom)

  )

  (:goal (and
    (cleaned veggiecabbage#1)
  ))
)
