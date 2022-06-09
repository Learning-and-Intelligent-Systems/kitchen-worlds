
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

	;; the following will be reconstructed and will be ignored by pddlstream
	wconf168
    wconf136
    wconf816
    wconf584
    wconf0
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

	(bottle oilbottle#1)
	(bottle vinegarbottle#1)

	(graspable oilbottle#1)
	(graspable meatturkeyleg#1)
	(graspable vinegarbottle#1)
	(graspable braiserlid#1)
	(graspable veggiecabbage#1)
	(oilbottle oilbottle#1)

	(edible veggiecabbage#1)
	(edible meatturkeyleg#1)

	(braiserlid braiserlid#1)

	(braiserbody braiserbody#1)

	(meatturkeyleg meatturkeyleg#1)
	(oftype vinegarbottle#1 @bottle)
	(oftype meatturkeyleg#1 @edible)
	(oftype veggiecabbage#1 @edible)
	(oftype oilbottle#1 @bottle)
	(surface counter#1::indigo_tmp)
	(surface counter#1::front_left_stove)
	(surface braiserbody#1::braiser_bottom)
	(surface counter#1::hitman_tmp)
	(veggiecabbage veggiecabbage#1)
	(vinegarbottle vinegarbottle#1)

	(space counter#1::hitman_drawer_top)
	(space counter#1::sektion)
	(space counter#1::dagger)

	(door counter#1::chewie_door_right_joint)
	(door counter#1::dagger_door_right_joint)
	(door counter#1::dagger_door_left_joint)
	(door counter#1::chewie_door_left_joint)
	(joint counter#1::dagger_door_left_joint)
	(joint counter#1::chewie_door_left_joint)
	(joint counter#1::dagger_door_right_joint)
	(joint counter#1::chewie_door_right_joint)
	(seconf q832=(0.9, 8, 0.7, 0, -1.571, 0))

	(atseconf q832=(0.9, 8, 0.7, 0, -1.571, 0))

	(containable braiserlid#1 counter#1::dagger)
	(containable braiserlid#1 counter#1::sektion)
	(containable vinegarbottle#1 counter#1::hitman_drawer_top)
	(containable vinegarbottle#1 counter#1::sektion)
	(containable veggiecabbage#1 counter#1::sektion)
	(containable meatturkeyleg#1 counter#1::hitman_drawer_top)
	(containable vinegarbottle#1 counter#1::dagger)
	(containable veggiecabbage#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::sektion)
	(containable braiserlid#1 counter#1::hitman_drawer_top)
	(containable meatturkeyleg#1 counter#1::dagger)
	(containable oilbottle#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::sektion)
	(containable veggiecabbage#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::dagger)

	(stackable oilbottle#1 counter#1::hitman_tmp)
	(stackable vinegarbottle#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable braiserlid#1 counter#1::hitman_tmp)
	(stackable oilbottle#1 braiserbody#1::braiser_bottom)
	(stackable vinegarbottle#1 counter#1::front_left_stove)
	(stackable veggiecabbage#1 counter#1::front_left_stove)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable vinegarbottle#1 counter#1::hitman_tmp)
	(stackable veggiecabbage#1 counter#1::hitman_tmp)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable vinegarbottle#1 braiserbody#1::braiser_bottom)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable oilbottle#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 counter#1::hitman_tmp)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)

	(originalseconf q832=(0.9, 8, 0.7, 0, -1.571, 0))

	(atposition counter#1::chewie_door_right_joint pstn1=0)
	(atposition counter#1::chewie_door_left_joint pstn0=0)
	(atposition counter#1::dagger_door_left_joint pstn3=0)
	(atposition counter#1::dagger_door_right_joint pstn2=0)
	(isjointto counter#1::chewie_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)
	(isjointto counter#1::dagger_door_left_joint counter#1)
	(isjointto counter#1::dagger_door_right_joint counter#1)

	(position counter#1::dagger_door_left_joint pstn7=-1.57)
	(position counter#1::chewie_door_left_joint pstn0=0)
	(position counter#1::chewie_door_right_joint pstn5=1.57)
	(position counter#1::chewie_door_right_joint pstn1=0)
	(position counter#1::dagger_door_right_joint pstn6=1.57)
	(position counter#1::dagger_door_left_joint pstn3=0)
	(position counter#1::dagger_door_right_joint pstn2=0)
	(position counter#1::chewie_door_left_joint pstn4=-1.57)

	(atpose braiserlid#1 p2=(0.566, 8.18, 0.821, 0, 0, 3.104))
	(atpose meatturkeyleg#1 p3=(0.849, 8.837, 0.844, 0, 0, 2.786))
	(atpose vinegarbottle#1 p0=(0.343, 8.698, 1.532, 0, 0, 2.222))
	(atpose veggiecabbage#1 p4=(0.642, 8.531, 0.87, 0, 0, 1.651))
	(atpose oilbottle#1 p1=(0.407, 8.642, 1.553, 0, 0, 1.09))

	(pose meatturkeyleg#1 p3=(0.849, 8.837, 0.844, 0, 0, 2.786))
	(pose braiserlid#1 p2=(0.566, 8.18, 0.821, 0, 0, 3.104))
	(pose oilbottle#1 p1=(0.407, 8.642, 1.553, 0, 0, 1.09))
	(pose vinegarbottle#1 p0=(0.343, 8.698, 1.532, 0, 0, 2.222))
	(pose veggiecabbage#1 p4=(0.642, 8.531, 0.87, 0, 0, 1.651))

	(isclosedposition counter#1::chewie_door_right_joint pstn1=0)
	(isclosedposition counter#1::dagger_door_left_joint pstn3=0)
	(isclosedposition counter#1::dagger_door_right_joint pstn2=0)
	(isclosedposition counter#1::chewie_door_left_joint pstn0=0)

	(isopenedposition counter#1::chewie_door_left_joint pstn4=-1.57)
	(isopenedposition counter#1::dagger_door_left_joint pstn7=-1.57)
	(isopenedposition counter#1::chewie_door_right_joint pstn5=1.57)
	(isopenedposition counter#1::dagger_door_right_joint pstn6=1.57)

	(contained oilbottle#1 p1=(0.407, 8.642, 1.553, 0, 0, 1.09) counter#1::dagger)
	(contained vinegarbottle#1 p0=(0.343, 8.698, 1.532, 0, 0, 2.222) counter#1::dagger)

	(wconf wconf168)
	(inwconf wconf168)
	(wconf wconf136)
	(wconf wconf816)
	(wconf wconf584)
	(wconf wconf0)
	(newwconfpst wconf168 counter#1::chewie_door_left_joint pstn4=-1.57 wconf136)
	(newwconfpst wconf168 counter#1::dagger_door_right_joint pstn6=1.57 wconf816)
	(newwconfpst wconf168 counter#1::dagger_door_left_joint pstn7=-1.57 wconf584)
	(newwconfpst wconf168 counter#1::chewie_door_right_joint pstn5=1.57 wconf0)

	(supported meatturkeyleg#1 p3=(0.849, 8.837, 0.844, 0, 0, 2.786) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p4=(0.642, 8.531, 0.87, 0, 0, 1.651) counter#1::indigo_tmp)

  )

  (:goal (and
    (handlegrasped hand counter#1::chewie_door_left_joint)
    (debug2)
    ; (graspedhandle counter#1::chewie_door_left_joint)
	; (graspedhandle counter#1::dagger_door_left_joint)
	; (storedinspace @bottle counter#1::sektion)
  ))
)
