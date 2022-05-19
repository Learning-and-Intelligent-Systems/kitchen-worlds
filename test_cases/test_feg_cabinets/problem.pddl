
(define
  (problem test_feg_cabinets)
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
	(surface counter#1::hitman_tmp)
	(surface braiserbody#1::braiser_bottom)
	(surface counter#1::indigo_tmp)
	(surface counter#1::front_left_stove)

	(space counter#1::hitman_drawer_top)
	(space counter#1::dagger)
	(space counter#1::sektion)

	(door counter#1::chewie_door_left_joint)
	(door counter#1::chewie_door_right_joint)

	(joint counter#1::chewie_door_left_joint)
	(joint counter#1::chewie_door_right_joint)
	(seconf q240=(0.9, 8, 0.7, 0, -1.571, 0))

	(atseconf q240=(0.9, 8, 0.7, 0, -1.571, 0))

	(containable meatturkeyleg#1 counter#1::sektion)
	(containable oilbottle#1 counter#1::hitman_drawer_top)
	(containable braiserlid#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::sektion)
	(containable braiserlid#1 counter#1::sektion)
	(containable meatturkeyleg#1 counter#1::dagger)
	(containable oilbottle#1 counter#1::dagger)
	(containable braiserlid#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::hitman_drawer_top)

	(originalseconf q240=(0.9, 8, 0.7, 0, -1.571, 0))
	(stackable meatturkeyleg#1 counter#1::hitman_tmp)
	(stackable oilbottle#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable oilbottle#1 counter#1::hitman_tmp)
	(stackable oilbottle#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::hitman_tmp)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)

	(position counter#1::chewie_door_left_joint pstn0=0)
	(position counter#1::chewie_door_right_joint pstn1=0)

	(atposition counter#1::chewie_door_right_joint pstn1=0)
	(atposition counter#1::chewie_door_left_joint pstn0=0)
	(isjointto counter#1::chewie_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)

	(atpose braiserlid#1 p1=(0.55, 8.153, 0.821, 0, 0, 0.681))
	(atpose oilbottle#1 p0=(0.663, 7.164, 1.245, 0, 0, 1.925))
	(atpose meatturkeyleg#1 p2=(0.7, 8.552, 0.844, 0, 0, 0.875))
	(pose meatturkeyleg#1 p2=(0.7, 8.552, 0.844, 0, 0, 0.875))
	(pose braiserlid#1 p1=(0.55, 8.153, 0.821, 0, 0, 0.681))
	(pose oilbottle#1 p0=(0.663, 7.164, 1.245, 0, 0, 1.925))

	(isclosedposition counter#1::chewie_door_left_joint pstn0=0)
	(isclosedposition counter#1::chewie_door_right_joint pstn1=0)

	(supported meatturkeyleg#1 p2=(0.7, 8.552, 0.844, 0, 0, 0.875) counter#1::indigo_tmp)

  )

  (:goal (and
    (on oilbottle#1 counter#1::indigo_tmp)
  ))
)
        