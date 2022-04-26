
(define
  (problem kitchen_lunch)
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
	left
	meatturkeyleg#1
	microwave#1
	milkbottle#1
	oven#1
	oven#1::knob_joint_2
	right
	veggiecabbage#1
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm left)
	(arm right)

	(oven oven#1)

	(floor floor1)

	(basin basin#1)

	(handempty left)
	(handempty right)

	(faucet faucet#1)
	(fridge fridge#1)

	(controllable left)
	(counter counter#1)

	(microwave microwave#1)

	(edible veggiecabbage#1)
	(edible meatturkeyleg#1)

	(braiserlid braiserlid#1)
	(dishwasher dishwasher#1)
	(milkbottle milkbottle#1)

	(braiserbody braiserbody#1)
	(graspable veggiecabbage#1)
	(graspable braiserlid#1)
	(graspable meatturkeyleg#1)
	(knob oven#1::knob_joint_2)
	(knob faucet#1::joint_faucet_1)
	(knob faucet#1::joint_faucet_0)

	(bconf q344=(2.5, 6, 3.142))
	(door fridge#1::fridge_door)

	(joint fridge#1::fridge_door)
	(joint oven#1::knob_joint_2)
	(joint faucet#1::joint_faucet_1)
	(joint faucet#1::joint_faucet_0)
	(joint oven#1::knob_joint_2)
	(joint faucet#1::joint_faucet_1)
	(joint faucet#1::joint_faucet_0)

	(atbconf q344=(2.5, 6, 3.142))

	(meatturkeyleg meatturkeyleg#1)
	(veggiecabbage veggiecabbage#1)

	(surface fridge#1::shelf_bottom)
	(surface counter#1::indigo_tmp)
	(surface basin#1::basin_bottom)
	(surface braiserbody#1::braiser_bottom)
	(surface counter#1::front_left_stove)

	(cleaningsurface basin#1::basin_bottom)

	(isjointto fridge#1::fridge_door fridge#1)
	(isjointto oven#1::knob_joint_2 oven#1)
	(isjointto faucet#1::joint_faucet_1 faucet#1)
	(isjointto faucet#1::joint_faucet_0 faucet#1)
	(position fridge#1::fridge_door pstn0=1.2)
	(position oven#1::knob_joint_2 pstn1=0)
	(position faucet#1::joint_faucet_1 pstn2=0)
	(position faucet#1::joint_faucet_0 pstn3=0)

	(atposition fridge#1::fridge_door pstn0=1.2)
	(atposition oven#1::knob_joint_2 pstn1=0)
	(atposition faucet#1::joint_faucet_1 pstn2=0)
	(atposition faucet#1::joint_faucet_0 pstn3=0)

	(heatingsurface braiserbody#1::braiser_bottom)

	(isclosedposition fridge#1::fridge_door pstn0=1.2)
	(isclosedposition oven#1::knob_joint_2 pstn1=0)
	(isclosedposition faucet#1::joint_faucet_1 pstn2=0)
	(isclosedposition faucet#1::joint_faucet_0 pstn3=0)
	(pose veggiecabbage#1 p5=(0.7, 4.8, 0.838, 2.259))
	(pose braiserlid#1 p6=(0.55, 8.227, 0.821, 1.458))
	(pose meatturkeyleg#1 p7=(0.7, 8.865, 0.844, 0.045))
	(stackable veggiecabbage#1 fridge#1::shelf_bottom)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 counter#1::front_left_stove)
	(stackable braiserlid#1 fridge#1::shelf_bottom)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 basin#1::basin_bottom)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 fridge#1::shelf_bottom)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 basin#1::basin_bottom)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)

	(atpose veggiecabbage#1 p5=(0.7, 4.8, 0.838, 2.259))
	(atpose braiserlid#1 p6=(0.55, 8.227, 0.821, 1.458))
	(atpose meatturkeyleg#1 p7=(0.7, 8.865, 0.844, 0.045))

	(linkpose fridge#1::fridge_door lp0=(0.767, 5.12, 1.031, 2.771))
	(linkpose oven#1::knob_joint_2 lp1=(0.293, 8.187, 0.993, 1.571))
	(linkpose faucet#1::joint_faucet_1 lp2=(0.3, 5.6, 0.985, 1.571))
	(linkpose faucet#1::joint_faucet_0 lp3=(0.3, 5.6, 0.985, 1.571))

	(controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)
	(controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)

	(atlinkpose fridge#1::fridge_door lp0=(0.767, 5.12, 1.031, 2.771))
	(atlinkpose oven#1::knob_joint_2 lp1=(0.293, 8.187, 0.993, 1.571))
	(atlinkpose faucet#1::joint_faucet_1 lp2=(0.3, 5.6, 0.985, 1.571))
	(atlinkpose faucet#1::joint_faucet_0 lp3=(0.3, 5.6, 0.985, 1.571))

	(aconf left aq536=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq440=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(ataconf left aq536=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq440=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(defaultconf left aq536=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultconf right aq440=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

	(supported meatturkeyleg#1 p7=(0.7, 8.865, 0.844, 0.045) counter#1::indigo_tmp)

  )

  (:goal (and
    (on meatturkeyleg#1 braiserbody#1::braiser_bottom)
  ))
)
        