
(define
  (problem test_feg_cabinets_rearrange_1)
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
	wconf176
	wconf312
	wconf352
	wconf512
	wconf992
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm hand)

	(oven oven#1)

	(floor floor1)

	(handempty hand)
	(wconf wconf176)
	(wconf wconf512)
	(wconf wconf352)
	(wconf wconf312)
	(wconf wconf992)

	(inwconf wconf352)

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

	(edible meatturkeyleg#1)
	(edible veggiecabbage#1)

	(braiserlid braiserlid#1)
	(space counter#1::dagger)
	(space counter#1::sektion)
	(space counter#1::hitman_drawer_top)

	(braiserbody braiserbody#1)

	(oftype oilbottle#1 @bottle)
	(oftype veggiecabbage#1 @edible)
	(oftype meatturkeyleg#1 @edible)
	(oftype vinegarbottle#1 @bottle)

	(meatturkeyleg meatturkeyleg#1)
	(surface counter#1::indigo_tmp)
	(surface counter#1::front_left_stove)
	(surface counter#1::hitman_tmp)
	(surface braiserbody#1::braiser_bottom)
	(veggiecabbage veggiecabbage#1)
	(vinegarbottle vinegarbottle#1)

	(seconf q16=(0.9, 8, 0.7, 0, -1.571, 0))

	(door counter#1::dagger_door_right_joint)
	(door counter#1::chewie_door_right_joint)
	(door counter#1::dagger_door_left_joint)
	(door counter#1::chewie_door_left_joint)

	(atseconf q16=(0.9, 8, 0.7, 0, -1.571, 0))

	(containable vinegarbottle#1 counter#1::sektion)
	(containable braiserlid#1 counter#1::sektion)
	(containable oilbottle#1 counter#1::hitman_drawer_top)
	(containable veggiecabbage#1 counter#1::sektion)
	(containable meatturkeyleg#1 counter#1::hitman_drawer_top)
	(containable oilbottle#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::dagger)
	(containable vinegarbottle#1 counter#1::hitman_drawer_top)
	(containable braiserlid#1 counter#1::hitman_drawer_top)
	(containable veggiecabbage#1 counter#1::hitman_drawer_top)
	(containable vinegarbottle#1 counter#1::dagger)
	(containable braiserlid#1 counter#1::dagger)
	(containable oilbottle#1 counter#1::sektion)
	(containable veggiecabbage#1 counter#1::dagger)
	(containable meatturkeyleg#1 counter#1::sektion)
	(originalseconf q16=(0.9, 8, 0.7, 0, -1.571, 0))

	(stackable veggiecabbage#1 counter#1::hitman_tmp)
	(stackable vinegarbottle#1 counter#1::front_left_stove)
	(stackable meatturkeyleg#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::front_left_stove)
	(stackable vinegarbottle#1 counter#1::hitman_tmp)
	(stackable veggiecabbage#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 counter#1::hitman_tmp)
	(stackable oilbottle#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 counter#1::indigo_tmp)
	(stackable vinegarbottle#1 braiserbody#1::braiser_bottom)
	(stackable braiserlid#1 braiserbody#1::braiser_bottom)
	(stackable oilbottle#1 counter#1::hitman_tmp)
	(stackable veggiecabbage#1 counter#1::indigo_tmp)
	(stackable oilbottle#1 counter#1::front_left_stove)
	(stackable oilbottle#1 braiserbody#1::braiser_bottom)
	(stackable meatturkeyleg#1 counter#1::front_left_stove)
	(stackable vinegarbottle#1 counter#1::indigo_tmp)
	(stackable meatturkeyleg#1 counter#1::hitman_tmp)
	(stackable braiserlid#1 counter#1::indigo_tmp)
	(stackable veggiecabbage#1 counter#1::front_left_stove)

	(isjointto counter#1::dagger_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_left_joint counter#1)
	(isjointto counter#1::chewie_door_right_joint counter#1)
	(isjointto counter#1::dagger_door_right_joint counter#1)

	(position counter#1::dagger_door_left_joint pstn3=-1.57)
	(position counter#1::chewie_door_right_joint pstn1=1.57)
	(position counter#1::chewie_door_left_joint pstn4=0)
	(position counter#1::dagger_door_right_joint pstn2=1.57)
	(position counter#1::dagger_door_left_joint pstn7=0)
	(position counter#1::chewie_door_right_joint pstn5=0)
	(position counter#1::chewie_door_left_joint pstn0=-1.57)
	(position counter#1::dagger_door_right_joint pstn6=0)

	(pose braiserlid#1 p2=(0.559, 8.205, 0.821, 0, 0, 0.918))
	(pose oilbottle#1 p1=(0.263, 8.814, 1.55, 0, 0, 1.1))
	(pose meatturkeyleg#1 p3=(0.355, 8.905, 0.844, 0, 0, 1.233))
	(pose vinegarbottle#1 p0=(0.377, 9.107, 1.532, 0, 0, 2.325))
	(pose veggiecabbage#1 p4=(0.742, 9.066, 0.87, 0, 0, 2.575))

	(atposition counter#1::chewie_door_right_joint pstn1=1.57)
	(atposition counter#1::dagger_door_left_joint pstn3=-1.57)
	(atposition counter#1::dagger_door_right_joint pstn2=1.57)
	(atposition counter#1::chewie_door_left_joint pstn0=-1.57)

	(isopenedposition counter#1::chewie_door_right_joint pstn5=0)
	(isopenedposition counter#1::dagger_door_left_joint pstn7=0)
	(isopenedposition counter#1::chewie_door_left_joint pstn4=0)
	(isopenedposition counter#1::dagger_door_right_joint pstn6=0)

	(atpose vinegarbottle#1 p0=(0.377, 9.107, 1.532, 0, 0, 2.325))
	(atpose braiserlid#1 p2=(0.559, 8.205, 0.821, 0, 0, 0.918))
	(atpose oilbottle#1 p1=(0.263, 8.814, 1.55, 0, 0, 1.1))
	(atpose veggiecabbage#1 p4=(0.742, 9.066, 0.87, 0, 0, 2.575))
	(atpose meatturkeyleg#1 p3=(0.355, 8.905, 0.844, 0, 0, 1.233))

	(isclosedposition counter#1::chewie_door_left_joint pstn0=-1.57)
	(isclosedposition counter#1::dagger_door_left_joint pstn3=-1.57)
	(isclosedposition counter#1::chewie_door_right_joint pstn1=1.57)
	(isclosedposition counter#1::dagger_door_right_joint pstn2=1.57)

	(newwconfpst wconf352 counter#1::dagger_door_right_joint pstn6=0 wconf992)
	(newwconfpst wconf352 counter#1::chewie_door_right_joint pstn5=0 wconf176)
	(newwconfpst wconf352 counter#1::chewie_door_left_joint pstn4=0 wconf312)
	(newwconfpst wconf352 counter#1::dagger_door_left_joint pstn7=0 wconf512)

	(contained oilbottle#1 p1=(0.263, 8.814, 1.55, 0, 0, 1.1) counter#1::dagger)
	(contained vinegarbottle#1 p0=(0.377, 9.107, 1.532, 0, 0, 2.325) counter#1::dagger)

	(supported meatturkeyleg#1 p3=(0.355, 8.905, 0.844, 0, 0, 1.233) counter#1::indigo_tmp)
	(supported veggiecabbage#1 p4=(0.742, 9.066, 0.87, 0, 0, 2.575) counter#1::indigo_tmp)

  )

  (:goal (and
    (in oilbottle#1 counter#1::sektion)
  ))
)
        