
(define
  (problem test_feg_cook_only)
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
	wconf240
	wconf472
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
	(wconf wconf472)
	(wconf wconf240)

	(faucet faucet#1)
	(fridge fridge#1)

	(inwconf wconf472)

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

	(meatturkeyleg meatturkeyleg#1)
	(surface basin#1::basin_bottom)
	(surface fridge#1::shelf_bottom)
	(surface braiserbody#1::braiser_bottom)
	(surface counter#1::indigo_tmp)
	(surface counter#1::front_left_stove)
	(veggiecabbage veggiecabbage#1)

	(joint faucet#1::joint_faucet_0)
	(joint faucet#1::joint_faucet_1)
	(joint oven#1::knob_joint_2)
	(oftype meatturkeyleg#1 @edible)
	(oftype veggiecabbage#1 @edible)

	(cleaningsurface basin#1::basin_bottom)
	(isjointto oven#1::knob_joint_2 oven#1)
	(isjointto fridge#1::fridge_door fridge#1)
	(isjointto faucet#1::joint_faucet_0 faucet#1)
	(isjointto faucet#1::joint_faucet_1 faucet#1)

	(seconf q328=(0.9, 8, 0.7, 0, -1.571, 0))

	(position fridge#1::fridge_door pstn0=1.5)
	(position faucet#1::joint_faucet_1 pstn2=0)
	(position fridge#1::fridge_door pstn6=1.592)
	(position oven#1::knob_joint_2 pstn1=0)
	(position faucet#1::joint_faucet_0 pstn3=0)

	(atseconf q328=(0.9, 8, 0.7, 0, -1.571, 0))

	(atposition faucet#1::joint_faucet_1 pstn2=0)
	(atposition faucet#1::joint_faucet_0 pstn3=0)
	(atposition oven#1::knob_joint_2 pstn1=0)
	(atposition fridge#1::fridge_door pstn0=1.5)

	(heatingsurface braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable veggiecabbage#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 fridge#1::shelf_bottom)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable braiserlid#1 fridge#1::shelf_bottom)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 basin#1::basin_bottom)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 fridge#1::shelf_bottom)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 basin#1::basin_bottom)

	(isclosedposition oven#1::knob_joint_2 pstn1=0)
	(isclosedposition faucet#1::joint_faucet_0 pstn3=0)
	(isclosedposition fridge#1::fridge_door pstn0=1.5)
	(isclosedposition faucet#1::joint_faucet_1 pstn2=0)

	(originalseconf q328=(0.9, 8, 0.7, 0, -1.571, 0))

	(isopenedposition fridge#1::fridge_door pstn6=1.592)

	(pose meatturkeyleg#1 p1=(0.535, 8.876, 0.844, 0, 0, 1.813))
	(pose veggiecabbage#1 p2=(0.52, 4.964, 0.838, 0, 0, 1.992))
	(pose braiserlid#1 p0=(0.586, 8.176, 0.821, 0, 0, 2.008))

	(atpose veggiecabbage#1 p2=(0.52, 4.964, 0.838, 0, 0, 1.992))
	(atpose braiserlid#1 p0=(0.586, 8.176, 0.821, 0, 0, 2.008))
	(atpose meatturkeyleg#1 p1=(0.535, 8.876, 0.844, 0, 0, 1.813))
	(controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)
	(controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)

	(newwconfpst wconf472 fridge#1::fridge_door pstn6=1.592 wconf240)

	(supported meatturkeyleg#1 p1=(0.535, 8.876, 0.844, 0, 0, 1.813) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p2=(0.52, 4.964, 0.838, 0, 0, 1.992) fridge#1::shelf_bottom)

  )

  (:goal (and
    (cleaned veggiecabbage#1)
  ))
)
        