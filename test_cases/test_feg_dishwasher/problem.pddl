
(define
  (problem test_feg_dishwasher)
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
	dishwasher#1::dishwasher_door
	dishwasher#1::surface_plate_left
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
	platefat#1
	veggiecabbage#1
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

	(faucet faucet#1)
	(fridge fridge#1)

	(controllable hand)
	(counter counter#1)

	(platefat platefat#1)

	(containobj platefat#1)

	(edible meatturkeyleg#1)
	(edible veggiecabbage#1)
	(graspable braiserlid#1)
	(graspable veggiecabbage#1)
	(graspable meatturkeyleg#1)
	(graspable platefat#1)

	(braiserlid braiserlid#1)
	(dishwasher dishwasher#1)

	(braiserbody braiserbody#1)

	(door fridge#1::fridge_door)

	(knob faucet#1::joint_faucet_1)
	(knob oven#1::knob_joint_2)
	(knob faucet#1::joint_faucet_0)
	(meatturkeyleg meatturkeyleg#1)
	(oftype meatturkeyleg#1 edible)
	(oftype veggiecabbage#1 edible)
	(veggiecabbage veggiecabbage#1)

	(joint faucet#1::joint_faucet_0)
	(joint faucet#1::joint_faucet_1)
	(joint oven#1::knob_joint_2)
	(joint dishwasher#1::dishwasher_door)
	(joint fridge#1::fridge_door)

	(surface counter#1::front_left_stove)
	(surface basin#1::basin_bottom)
	(surface dishwasher#1::surface_plate_left)
	(surface fridge#1::shelf_bottom)
	(surface counter#1::indigo_tmp)
	(surface braiserbody#1::braiser_bottom)
	(surface platefat#1)

	(drawer dishwasher#1::dishwasher_door)

	(cleaningsurface basin#1::basin_bottom)
	(isjointto oven#1::knob_joint_2 oven#1)
	(isjointto fridge#1::fridge_door fridge#1)
	(isjointto faucet#1::joint_faucet_0 faucet#1)
	(isjointto faucet#1::joint_faucet_1 faucet#1)
	(isjointto dishwasher#1::dishwasher_door dishwasher#1)

	(seconf q344=(0.9, 8, 0.7, 0, -1.571, 0))

	(atseconf q344=(0.9, 8, 0.7, 0, -1.571, 0))
	(position faucet#1::joint_faucet_1 pstn4=0)
	(position oven#1::knob_joint_2 pstn2=0)
	(position faucet#1::joint_faucet_0 pstn3=0)
	(position fridge#1::fridge_door pstn8=1.956)
	(position dishwasher#1::dishwasher_door pstn5=0.49)
	(position fridge#1::fridge_door pstn1=1.5)
	(position dishwasher#1::dishwasher_door pstn0=0)

	(heatingsurface braiserbody#1::braiser_bottom)

	(originalseconf q344=(0.9, 8, 0.7, 0, -1.571, 0))

	(atposition dishwasher#1::dishwasher_door pstn0=0)
	(atposition fridge#1::fridge_door pstn1=1.5)
	(atposition faucet#1::joint_faucet_1 pstn4=0)
	(atposition oven#1::knob_joint_2 pstn2=0)
	(atposition faucet#1::joint_faucet_0 pstn3=0)

	(isclosedposition faucet#1::joint_faucet_1 pstn4=0)
	(isclosedposition oven#1::knob_joint_2 pstn2=0)
	(isclosedposition fridge#1::fridge_door pstn1=1.5)
	(isclosedposition dishwasher#1::dishwasher_door pstn0=0)
	(isclosedposition faucet#1::joint_faucet_0 pstn3=0)

	(isopenedposition fridge#1::fridge_door pstn8=1.956)
	(isopenedposition dishwasher#1::dishwasher_door pstn5=0.49)

	(pose platefat#1 p4=(0.97, 6.23, 0.512, 0, 0, 3.142))
	(pose meatturkeyleg#1 p2=(0.544, 9.033, 0.844, 0, 0, 3.111))
	(pose veggiecabbage#1 p3=(0.407, 4.749, 0.838, 0, 0, 0.681))
	(pose braiserlid#1 p1=(0.589, 8.18, 0.821, 0, 0, 0.068))
	(pose platefat#1 p0=(0.48, 6.23, 0.512, 0, 0, 3.142))

	(atattachment platefat#1 dishwasher#1::dishwasher_door)

	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 platefat#1)
	(stackable veggiecabbage#1 basin#1::basin_bottom)
	(stackable platefat#1 platefat#1)
	(stackable platefat#1 basin#1::basin_bottom)
	(stackable braiserlid#1 fridge#1::shelf_bottom)
	(stackable veggiecabbage#1 fridge#1::shelf_bottom)
	(stackable braiserlid#1 dishwasher#1::surface_plate_left)
	(stackable platefat#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 dishwasher#1::surface_plate_left)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 basin#1::basin_bottom)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable platefat#1 fridge#1::shelf_bottom)
	(stackable braiserlid#1 platefat#1)
	(stackable veggiecabbage#1 platefat#1)
	(stackable platefat#1 dishwasher#1::surface_plate_left)
	(stackable braiserlid#1 basin#1::basin_bottom)
	(stackable platefat#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 fridge#1::shelf_bottom)
	(stackable platefat#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 dishwasher#1::surface_plate_left)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)

	(atpose veggiecabbage#1 p3=(0.407, 4.749, 0.838, 0, 0, 0.681))
	(atpose braiserlid#1 p1=(0.589, 8.18, 0.821, 0, 0, 0.068))
	(atpose meatturkeyleg#1 p2=(0.544, 9.033, 0.844, 0, 0, 3.111))
	(atpose platefat#1 p0=(0.48, 6.23, 0.512, 0, 0, 3.142))

	(controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)
	(controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)

	(newwconfpst wconf584(5) fridge#1::fridge_door pstn8=1.956 wconf192(5))
	(newwconfpst wconf584(5) dishwasher#1::dishwasher_door pstn5=0.49 wconf136(5))

	(newposefromattachment platefat#1 p4=(0.97, 6.23, 0.512, 0, 0, 3.142) wconf136(5))

	(supported veggiecabbage#1 p3=(0.407, 4.749, 0.838, 0, 0, 0.681) fridge#1::shelf_bottom)
	(supported meatturkeyleg#1 p2=(0.544, 9.033, 0.844, 0, 0, 3.111) counter#1::indigo_tmp)

  )

  (:goal (and
    (graspedhandle dishwasher#1::dishwasher_door)
	(on platefat#1 counter#1::indigo_tmp)
  ))
)
        