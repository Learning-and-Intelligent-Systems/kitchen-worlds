
(define
  (problem test_feg_cabinets_rearrange)
  (:domain domain)

  (:objects
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
	floor1
	hand
	meatturkeyleg#1
	oilbottle#1
	oven#1
	veggiecabbage#1
	vinegarbottle#1
	wconf344
	wconf600
	wconf736
	wconf776
	wconf936
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(floor floor1)

	(handempty hand)
	(wconf wconf600)
	(wconf wconf936)
	(wconf wconf344)
	(wconf wconf776)
	(wconf wconf736)

	(inwconf wconf776)

	(controllable hand)

	(graspable braiserlid#1)
	(graspable veggiecabbage#1)
	(graspable oilbottle#1)
	(graspable meatturkeyleg#1)
	(graspable vinegarbottle#1)

	(space counter#1::sektion)
	(space counter#1::hitman_drawer_top)
	(space counter#1::dagger)

	(surface counter#1::indigo_tmp)
	(surface counter#1::front_left_stove)
	(surface counter#1::hitman_tmp)
	(surface braiserbody#1::braiser_bottom)

	(oftype veggiecabbage#1 @edible)
	(oftype oilbottle#1 @bottle)
	(oftype meatturkeyleg#1 @edible)
	(oftype vinegarbottle#1 @bottle)

	(door counter#1::dagger_door_left_joint)
	(door counter#1::chewie_door_left_joint)
	(door counter#1::dagger_door_right_joint)
	(door counter#1::chewie_door_right_joint)

	(seconf q440=(0.9, 8, 0.7, 0, -1.571, 0))

	(atseconf q440=(0.9, 8, 0.7, 0, -1.571, 0))

	(originalseconf q440=(0.9, 8, 0.7, 0, -1.571, 0))
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

	(position counter#1::chewie_door_right_joint pstn5=0)
	(position counter#1::dagger_door_left_joint pstn7=0)
	(position counter#1::dagger_door_right_joint pstn2=1.57)
	(position counter#1::chewie_door_left_joint pstn0=-1.57)
	(position counter#1::dagger_door_right_joint pstn6=0)
	(position counter#1::dagger_door_left_joint pstn3=-1.57)
	(position counter#1::chewie_door_left_joint pstn4=0)
	(position counter#1::chewie_door_right_joint pstn1=1.57)

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

	(isjointto counter#1::dagger_door_right_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)
	(isjointto counter#1::dagger_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_left_joint counter#1)
	(pose oilbottle#1 p1=(0.324, 8.944, 1.555, 0, 0, 1.615))
	(pose braiserlid#1 p2=(0.57, 8.186, 0.821, 0, 0, 1.582))
	(pose vinegarbottle#1 p0=(0.322, 8.631, 1.526, 0, 0, 2.383))
	(pose veggiecabbage#1 p4=(0.436, 8.82, 0.87, 0, 0, 0.891))
	(pose meatturkeyleg#1 p3=(0.633, 8.702, 0.844, 0, 0, 2.914))

	(atpose oilbottle#1 p1=(0.324, 8.944, 1.555, 0, 0, 1.615))
	(atpose veggiecabbage#1 p4=(0.436, 8.82, 0.87, 0, 0, 0.891))
	(atpose meatturkeyleg#1 p3=(0.633, 8.702, 0.844, 0, 0, 2.914))
	(atpose vinegarbottle#1 p0=(0.322, 8.631, 1.526, 0, 0, 2.383))
	(atpose braiserlid#1 p2=(0.57, 8.186, 0.821, 0, 0, 1.582))
	(atposition counter#1::chewie_door_left_joint pstn0=-1.57)
	(atposition counter#1::dagger_door_left_joint pstn3=-1.57)
	(atposition counter#1::chewie_door_right_joint pstn1=1.57)
	(atposition counter#1::dagger_door_right_joint pstn2=1.57)

	(isopenedposition counter#1::chewie_door_left_joint pstn4=0)
	(isopenedposition counter#1::dagger_door_left_joint pstn7=0)
	(isopenedposition counter#1::dagger_door_right_joint pstn6=0)
	(isopenedposition counter#1::chewie_door_right_joint pstn5=0)

	(isclosedposition counter#1::dagger_door_right_joint pstn2=1.57)
	(isclosedposition counter#1::chewie_door_left_joint pstn0=-1.57)
	(isclosedposition counter#1::dagger_door_left_joint pstn3=-1.57)
	(isclosedposition counter#1::chewie_door_right_joint pstn1=1.57)

	(newwconfpst wconf776 counter#1::dagger_door_right_joint pstn6=0 wconf936)
	(newwconfpst wconf776 counter#1::chewie_door_right_joint pstn5=0 wconf600)
	(newwconfpst wconf776 counter#1::chewie_door_left_joint pstn4=0 wconf736)
	(newwconfpst wconf776 counter#1::dagger_door_left_joint pstn7=0 wconf344)

	(contained vinegarbottle#1 p0=(0.322, 8.631, 1.526, 0, 0, 2.383) counter#1::dagger)
	(contained oilbottle#1 p1=(0.324, 8.944, 1.555, 0, 0, 1.615) counter#1::dagger)

	(supported meatturkeyleg#1 p3=(0.633, 8.702, 0.844, 0, 0, 2.914) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p4=(0.436, 8.82, 0.87, 0, 0, 0.891) counter#1::indigo_tmp)

  )

  (:goal (and
    (storedinspace @bottle counter#1::sektion)
  ))
)
        