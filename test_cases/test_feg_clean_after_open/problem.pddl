
(define
  (problem test_feg_clean_after_open)
  (:domain domain)

  (:objects
    basin#1
	basin#1::basin_bottom
	braiserbody#1
	braiserbody#1::braiser_bottom
	braiserlid#1
	counter#1
	counter#1::front_left_stove
	counter#1::indigo_tmp
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
	wconf0
	wconf184
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(oven oven#1)

	(floor floor1)

	(basin basin#1)

	(handempty hand)
	(wconf wconf184)
	(wconf wconf0)

	(faucet faucet#1)
	(fridge fridge#1)

	(inwconf wconf184)

	(controllable hand)
	(counter counter#1)

	(edible veggiecabbage#1)
	(edible meatturkeyleg#1)

	(braiserlid braiserlid#1)
	(cleaned meatturkeyleg#1)
	(dishwasher dishwasher#1)

	(braiserbody braiserbody#1)
	(graspable meatturkeyleg#1)
	(graspable braiserlid#1)
	(graspable veggiecabbage#1)
	(knob oven#1::knob_joint_2)
	(knob faucet#1::joint_faucet_1)
	(knob faucet#1::joint_faucet_0)

	(door fridge#1::fridge_door)
	(joint oven#1::knob_joint_2)
	(joint faucet#1::joint_faucet_0)
	(joint faucet#1::joint_faucet_1)

	(meatturkeyleg meatturkeyleg#1)
	(veggiecabbage veggiecabbage#1)

	(oftype veggiecabbage#1 @edible)
	(oftype meatturkeyleg#1 @edible)

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

	(seconf q40=(0.9, 8, 0.7, 0, -1.571, 0))

	(atposition fridge#1::fridge_door pstn0=0)
	(atposition faucet#1::joint_faucet_0 pstn3=0)
	(atposition oven#1::knob_joint_2 pstn1=0)
	(atposition faucet#1::joint_faucet_1 pstn2=0)
	(atseconf q40=(0.9, 8, 0.7, 0, -1.571, 0))

	(isjointto faucet#1::joint_faucet_1 faucet#1)
	(isjointto oven#1::knob_joint_2 oven#1)
	(isjointto fridge#1::fridge_door fridge#1)
	(isjointto faucet#1::joint_faucet_0 faucet#1)

	(heatingsurface braiserbody#1::braiser_bottom)

	(originalseconf q40=(0.9, 8, 0.7, 0, -1.571, 0))

	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 basin#1::basin_bottom)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 counter#1::front_left_stove)
	(stackable braiserlid#1 basin#1::basin_bottom)
	(stackable braiserlid#1 fridge#1::shelf_bottom)
	(stackable veggiecabbage#1 fridge#1::shelf_bottom)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable meatturkeyleg#1 fridge#1::shelf_bottom)

	(isclosedposition faucet#1::joint_faucet_1 pstn2=0)
	(isclosedposition faucet#1::joint_faucet_0 pstn3=0)
	(isclosedposition fridge#1::fridge_door pstn0=0)
	(isclosedposition oven#1::knob_joint_2 pstn1=0)

	(isopenedposition fridge#1::fridge_door pstn5=1.907)

	(atpose braiserlid#1 p0=(0.557, 8.211, 0.821, 0, 0, 0.627))
	(atpose meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126))
	(atpose veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536))

	(pose meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126))
	(pose veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536))
	(pose braiserlid#1 p0=(0.557, 8.211, 0.821, 0, 0, 0.627))

	(controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)
	(controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)

	(newwconfpst wconf184 fridge#1::fridge_door pstn5=1.907 wconf0)

	(supported meatturkeyleg#1 p1=(0.554, 8.706, 0.844, 0, 0, 3.126) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p2=(0.351, 4.911, 0.838, 0, 0, 1.536) fridge#1::shelf_bottom)

  )

  (:goal (and
    (cleaned veggiecabbage#1)
  ))
)
        