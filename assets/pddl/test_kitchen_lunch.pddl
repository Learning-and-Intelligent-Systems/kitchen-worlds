(define
  (problem test_kitchen_lunch)
  (:domain test_kitchen_lunch_domain)

  (:objects
    left
    right
    wconf736
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

    (wconf wconf736)
    (inwconf wconf736)

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

    (bconf q160=(2.5, 6, 3.142))
    (door fridge#1::fridge_door)

    (sdf::is-joint fridge#1::fridge_door)
    (sdf::is-joint oven#1::knob_joint_2)
    (sdf::is-joint faucet#1::joint_faucet_1)
    (sdf::is-joint faucet#1::joint_faucet_0)
    (sdf::is-joint oven#1::knob_joint_2)
    (sdf::is-joint faucet#1::joint_faucet_1)
    (sdf::is-joint faucet#1::joint_faucet_0)

    (atbconf q160=(2.5, 6, 3.142))

    (meatturkeyleg meatturkeyleg#1)
    (veggiecabbage veggiecabbage#1)

    (surface counter#1::front_left_stove)
    (surface fridge#1::shelf_bottom)
    (surface counter#1::indigo_tmp)
    (surface braiserbody#1::braiser_bottom)
    (surface basin#1::basin_bottom)

    (cleaningsurface basin#1::basin_bottom)

    (isjointto fridge#1::fridge_door fridge#1)
    (isjointto oven#1::knob_joint_2 oven#1)
    (isjointto faucet#1::joint_faucet_1 faucet#1)
    (isjointto faucet#1::joint_faucet_0 faucet#1)

    (position fridge#1::fridge_door pstn0=1.57)
    (position oven#1::knob_joint_2 pstn1=0)
    (position faucet#1::joint_faucet_1 pstn2=0)
    (position faucet#1::joint_faucet_0 pstn3=0)

    (atposition fridge#1::fridge_door pstn0=1.57)
    (atposition oven#1::knob_joint_2 pstn1=0)
    (atposition faucet#1::joint_faucet_1 pstn2=0)
    (atposition faucet#1::joint_faucet_0 pstn3=0)

    (heatingsurface braiserbody#1::braiser_bottom)

    (pose veggiecabbage#1 p0=(0.7, 4.8, 0.838, 1.443))
    (pose braiserlid#1 p1=(0.55, 8.186, 0.821, 1.382))
    (pose meatturkeyleg#1 p2=(0.7, 8.673, 0.844, 1.801))

    (isclosedposition fridge#1::fridge_door pstn0=1.57)
    (isclosedposition oven#1::knob_joint_2 pstn1=0)
    (isclosedposition faucet#1::joint_faucet_1 pstn2=0)
    (isclosedposition faucet#1::joint_faucet_0 pstn3=0)

    (atpose veggiecabbage#1 p0=(0.7, 4.8, 0.838, 1.443))
    (atpose braiserlid#1 p1=(0.55, 8.186, 0.821, 1.382))
    (atpose meatturkeyleg#1 p2=(0.7, 8.673, 0.844, 1.801))

    (stackable veggiecabbage#1 counter#1::front_left_stove)
    (stackable veggiecabbage#1 fridge#1::shelf_bottom)
    (stackable veggiecabbage#1 counter#1::indigo_tmp)
    (stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
    (stackable veggiecabbage#1 basin#1::basin_bottom)
    (stackable braiserlid#1 counter#1::front_left_stove)
    (stackable braiserlid#1 fridge#1::shelf_bottom)
    (stackable braiserlid#1 counter#1::indigo_tmp)
    (stackable braiserlid#1 braiserbody#1::braiser_bottom)
    (stackable braiserlid#1 basin#1::basin_bottom)
    (stackable meatturkeyleg#1 counter#1::front_left_stove)
    (stackable meatturkeyleg#1 fridge#1::shelf_bottom)
    (stackable meatturkeyleg#1 counter#1::indigo_tmp)
    (stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
    (stackable meatturkeyleg#1 basin#1::basin_bottom)

    (linkpose fridge#1::fridge_door lp0=(0.767, 5.12, 1.031, 3.141))
    (linkpose oven#1::knob_joint_2 lp1=(0.293, 8.187, 0.994, 1.571))
    (linkpose faucet#1::joint_faucet_1 lp2=(0.3, 5.6, 0.985, 1.571))
    (linkpose faucet#1::joint_faucet_0 lp3=(0.3, 5.6, 0.985, 1.571))

    (controlledby braiserbody#1::braiser_bottom oven#1::knob_joint_2)
    (controlledby basin#1::basin_bottom faucet#1::joint_faucet_0)

    (atlinkpose fridge#1::fridge_door lp0=(0.767, 5.12, 1.031, 3.141))
    (atlinkpose oven#1::knob_joint_2 lp1=(0.293, 8.187, 0.994, 1.571))
    (atlinkpose faucet#1::joint_faucet_1 lp2=(0.3, 5.6, 0.985, 1.571))
    (atlinkpose faucet#1::joint_faucet_0 lp3=(0.3, 5.6, 0.985, 1.571))

    (aconf left aq352=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
    (aconf right aq256=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

    (ataconf left aq352=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
    (ataconf right aq256=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

    (defaultconf left aq352=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
    (defaultconf right aq256=(-2.135, 1.296, -3.75, -0.15, -10000, -0.1, -10000))

    (supported meatturkeyleg#1 p2=(0.7, 8.673, 0.844, 1.801) counter#1::indigo_tmp)
  )

  (:goal (and
    (cooked meatturkeyleg#1)
    (cooked veggiecabbage#1)
  ))
)

