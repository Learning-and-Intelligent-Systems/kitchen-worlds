
(define
  (problem test_feg_pick)
  (:domain domain)

  (:objects
    braiserbody#1
	braiserbody#1::braiser_bottom
	braiserlid#1
	counter#1
	counter#1::chewie_door_left_joint
	counter#1::chewie_door_right_joint
	counter#1::dagger
	counter#1::front_left_stove
	counter#1::hitman_drawer_top
	counter#1::hitman_tmp
	counter#1::indigo_tmp
	counter#1::sektion
	floor1
	hand
	meatturkeyleg#1
	oilbottle#1
	oven#1
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(oven oven#1)

	(floor floor1)

	(handempty hand)

	(controllable hand)
	(counter counter#1)

	(graspable oilbottle#1)
	(graspable meatturkeyleg#1)
	(graspable braiserlid#1)
	(oilbottle oilbottle#1)

	(braiserlid braiserlid#1)

	(braiserbody braiserbody#1)

	(meatturkeyleg meatturkeyleg#1)

	(space counter#1::hitman_drawer_top)
	(space counter#1::sektion)
	(space counter#1::dagger)

	(surface counter#1::front_left_stove)
	(surface counter#1::indigo_tmp)
	(surface counter#1::hitman_tmp)
	(surface braiserbody#1::braiser_bottom)

	(door counter#1::chewie_door_left_joint)
	(door counter#1::chewie_door_right_joint)
	(seconf q48=(0.9, 8, 0.7, 0, -1.571, 0))

	(joint counter#1::chewie_door_left_joint)
	(joint counter#1::chewie_door_right_joint)

	(atseconf q48=(0.9, 8, 0.7, 0, -1.571, 0))

	(containable braiserlid#1 counter#1::sektion)
	(containable oilbottle#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::hitman_drawer_top)
	(containable braiserlid#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::sektion)
	(containable oilbottle#1 counter#1::hitman_drawer_top)
	(containable braiserlid#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::sektion)
	(containable meatturkeyleg#1 counter#1::dagger)

	(originalseconf q48=(0.9, 8, 0.7, 0, -1.571, 0))

	(isjointto counter#1::chewie_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::hitman_tmp)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 counter#1::front_left_stove)
	(stackable braiserlid#1 counter#1::hitman_tmp)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable oilbottle#1 counter#1::hitman_tmp)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)

	(pose braiserlid#1 p1=(0.55, 8.198, 0.821, 0, 0, 3.096))
	(pose meatturkeyleg#1 p2=(0.7, 8.9, 0.844, 0, 0, 2.427))
	(pose oilbottle#1 p0=(0.331, 6.904, 1.241, 0, 0, 0.418))

	(position counter#1::chewie_door_left_joint pstn0=-1.492)
	(position counter#1::chewie_door_right_joint pstn1=1.492)

	(atpose meatturkeyleg#1 p2=(0.7, 8.9, 0.844, 0, 0, 2.427))
	(atpose oilbottle#1 p0=(0.331, 6.904, 1.241, 0, 0, 0.418))
	(atpose braiserlid#1 p1=(0.55, 8.198, 0.821, 0, 0, 3.096))

	(atposition counter#1::chewie_door_left_joint pstn0=-1.492)
	(atposition counter#1::chewie_door_right_joint pstn1=1.492)

	(isclosedposition counter#1::chewie_door_left_joint pstn0=-1.492)
	(isclosedposition counter#1::chewie_door_right_joint pstn1=1.492)

	(supported meatturkeyleg#1 p2=(0.7, 8.9, 0.844, 0, 0, 2.427) counter#1::indigo_tmp)

  )

  (:goal (and
    (on oilbottle#1 counter#1::indigo_tmp)
  ))
)
        